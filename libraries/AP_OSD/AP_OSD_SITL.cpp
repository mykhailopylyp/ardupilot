/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
  OSD backend for SITL. Uses SFML media library. See
  https://www.sfml-dev.org/index.php

  To use install SFML libraries, and run sim_vehicle.py with --osd
  option. Then set OSD_TYPE to 2
 */
#ifdef WITH_SITL_OSD

#include "AP_OSD_SITL.h"
#include <AP_HAL/Util.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Scheduler.h>
#include <AP_ROMFS/AP_ROMFS.h>
#include <SITL/SITL.h>
#include <utility>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "pthread.h"

#include <cstdio>
#include <SFML/Graphics.hpp>

#include <AP_Notify/AP_Notify.h>

extern const AP_HAL::HAL &hal;

/*
  load *.bin font file, in MAX7456 format
 */
void AP_OSD_SITL::load_font(void)
{
    last_font = get_font_num();
    FileData *fd = load_font_data(last_font);
    if (fd == nullptr || fd->length != 54 * 256) {
        AP_HAL::panic("Bad font file");
    }
    for (uint16_t i=0; i<256; i++) {
        const uint8_t *c = &fd->data[i*54];
        // each pixel is 4 bytes, RGBA
        sf::Uint8 *pixels = NEW_NOTHROW sf::Uint8[char_width * char_height * 4];
        if (!font[i].create(char_width, char_height)) {
            AP_HAL::panic("Failed to create texture");
        }
        for (uint16_t y=0; y<char_height; y++) {
            for (uint16_t x=0; x<char_width; x++) {
                // 2 bits per pixel
                uint16_t bitoffset = (y*char_width+x)*2;
                uint8_t byteoffset = bitoffset / 8;
                uint8_t bitshift = 6-(bitoffset % 8);
                uint8_t v = (c[byteoffset] >> bitshift) & 3;
                sf::Uint8 *p = &pixels[(y*char_width+x)*4];
                switch (v) {
                case 0:
                    p[0] = 0;
                    p[1] = 0;
                    p[2] = 0;
                    p[3] = 255;
                    break;
                case 1:
                case 3:
                    p[0] = 0;
                    p[1] = 0;
                    p[2] = 0;
                    p[3] = 0;
                    break;
                case 2:
                    p[0] = 255;
                    p[1] = 255;
                    p[2] = 255;
                    p[3] = 255;
                    break;
                }

            }
        }
        font[i].update(pixels);
    }
    delete fd;
}

void AP_OSD_SITL::write(uint8_t x, uint8_t y, const char* text)
{
    if (y >= video_lines || text == nullptr) {
        return;
    }
    WITH_SEMAPHORE(mutex);

    while ((x < video_cols) && (*text != 0)) {
        getbuffer(buffer, y, x) = *text;
        ++text;
        ++x;
    }
}

void AP_OSD_SITL::clear(void)
{
    AP_OSD_Backend::clear();
    WITH_SEMAPHORE(mutex);
    memset(buffer, 0, video_cols*video_lines);
}

void AP_OSD_SITL::flush(void)
{
    counter++;
}

void AP_OSD_SITL::update_thread(void)
{
    load_font();

    // --- Determine frame dimensions (must match output video settings) ---
    const unsigned int width = video_cols * (char_width + char_spacing) * char_scale;
    const unsigned int height = video_lines * (char_height + char_spacing) * char_scale;
    printf("width: %d; height: %d\n", width, height);

    // --- Launch an FFmpeg process to accept raw RGB frames and output MJPEG ---
    char ffmpeg_cmd[512];
    // This command reads raw RGB video from stdin and writes an MJPEG stream to /tmp/osd.mjpg.
    // Adjust parameters (e.g., pixel_format, video_size, output destination) as needed.
    snprintf(ffmpeg_cmd, sizeof(ffmpeg_cmd),
             "ffmpeg -y -f rawvideo -pixel_format rgb24 -video_size %ux%u -i - "
             "-f mjpeg -q:v 5 /tmp/osd.mjpg",
             width, height);
    
    FILE *ffmpeg_pipe = popen(ffmpeg_cmd, "w");
    if (!ffmpeg_pipe) {
        AP_HAL::panic("Failed to launch FFmpeg process");
        return;
    }

    // --- Create the SFML window for on-screen display ---
    {
        WITH_SEMAPHORE(AP::notify().sf_window_mutex);
        w = NEW_NOTHROW sf::RenderWindow(sf::VideoMode(width, height), "OSD");
    }
    if (!w) {
        AP_HAL::panic("Unable to create OSD window");
        pclose(ffmpeg_pipe);
        return;
    }

    // Our FFmpeg process expects raw RGB data (3 bytes per pixel)
    size_t frame_size = width * height * 3;
    uint8_t *frame_buffer = new uint8_t[frame_size];

    // --- Create an off-screen render texture ---
    sf::RenderTexture renderTexture;
    if (!renderTexture.create(width, height)) {
        AP_HAL::panic("Unable to create render texture");
        delete[] frame_buffer;
        pclose(ffmpeg_pipe);
        return;
    }

    // Main update loop
    while (true) {
        WITH_SEMAPHORE(AP::notify().sf_window_mutex);
        sf::Event event;
        while (w->pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                w->close();
            }
        }
        if (!w->isOpen()) {
            break;
        }
        if (counter != last_counter) {
            last_counter = counter;

            // Copy the current OSD buffer to a local buffer.
            uint8_t buffer2[video_lines][video_cols];
            {
                WITH_SEMAPHORE(mutex);
                memcpy(buffer2, buffer, sizeof(buffer2));
            }

            // --- Render to the off-screen texture ---
            renderTexture.clear();
            for (uint8_t y = 0; y < video_lines; y++) {
                for (uint8_t x = 0; x < video_cols; x++) {
                    uint16_t px = x * (char_width + char_spacing) * char_scale;
                    uint16_t py = y * (char_height + char_spacing) * char_scale;
                    sf::Sprite s;
                    uint8_t c = buffer2[y][x];
                    s.setTexture(font[c]);
                    s.setPosition(sf::Vector2f(px, py));
                    s.scale(sf::Vector2f(char_scale, char_scale));
                    renderTexture.draw(s);
                }
            }
            renderTexture.display();

            // --- Display the off-screen texture in the window ---
            w->clear();
            sf::Sprite renderSprite(renderTexture.getTexture());
            w->draw(renderSprite);
            w->display();

            if (last_font != get_font_num()) {
                load_font();
            }

            // --- Capture the rendered frame from the render texture ---
            sf::Image screenshot = renderTexture.getTexture().copyToImage();
            const sf::Uint8 *pixels = screenshot.getPixelsPtr();

            // Convert from RGBA (4 bytes per pixel) to RGB (3 bytes per pixel)
            for (unsigned int i = 0, j = 0; i < width * height; i++, j += 4) {
                uint8_t r = pixels[j];
                uint8_t g = pixels[j + 1];
                uint8_t b = pixels[j + 2];
                frame_buffer[i * 3]     = r;
                frame_buffer[i * 3 + 1] = g;
                frame_buffer[i * 3 + 2] = b;
            }

            // --- Write the raw RGB frame to the FFmpeg process ---
            size_t written = fwrite(frame_buffer, 1, frame_size, ffmpeg_pipe);
            if (written != frame_size) {
                fprintf(stderr, "Error writing frame to FFmpeg (written %zu bytes)\n", written);
            }
        }
        usleep(10000);  // Sleep 10ms between iterations
    }

    // Signal end-of-stream and cleanup
    delete[] frame_buffer;
    pclose(ffmpeg_pipe);
    if (w) {
        delete w;
        w = nullptr;
    }
}

// trampoline for update thread
void *AP_OSD_SITL::update_thread_start(void *obj)
{
    ((AP_OSD_SITL *)obj)->update_thread();
    return nullptr;
}

// initialise backend
bool AP_OSD_SITL::init(void)
{
    pthread_create(&thread, NULL, update_thread_start, this);
    return true;
}

AP_OSD_Backend *AP_OSD_SITL::probe(AP_OSD &osd)
{
    AP_OSD_SITL *backend = NEW_NOTHROW AP_OSD_SITL(osd);
    if (!backend) {
        return nullptr;
    }
    if (!backend->init()) {
        delete backend;
        return nullptr;
    }
    return backend;
}

AP_OSD_SITL::AP_OSD_SITL(AP_OSD &osd):
    AP_OSD_Backend(osd)
{
    const auto *_sitl = AP::sitl();
    video_lines = _sitl->osd_rows;
    video_cols = _sitl->osd_columns;
    buffer = (uint8_t *)malloc(video_lines*video_cols);
}

#endif // WITH_SITL_OSD
