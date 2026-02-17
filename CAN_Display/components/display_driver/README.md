# display_driver - Display Hardware Abstraction Layer

## Purpose

Provides a unified framebuffer-based interface for rendering to different display hardware. The gauge engine draws to an abstract surface; the display driver handles the hardware-specific transfer.

## Architecture

```
┌────────────────────────────┐
│     gauge_engine           │
│  "Draw gauge at (x,y,w,h)"│
└────────────┬───────────────┘
             │ framebuffer API
┌────────────┴───────────────┐
│     display_driver.h       │
│  - display_init()          │
│  - display_flush()         │
│  - display_set_pixel()     │
│  - display_fill_rect()     │
│  - display_draw_text()     │
└──┬─────────┬───────────────┘
   │         │
┌──┴────┐ ┌──┴────┐
│ILI9341│ │ST7789 │  (future: ILI9488, GC9A01, etc.)
│ SPI   │ │ SPI   │
└───────┘ └───────┘
```

## Display Capabilities

The driver exposes a common feature set:

```c
typedef struct {
    uint16_t width;
    uint16_t height;
    uint8_t  rotation;          // 0, 90, 180, 270
    uint16_t color_depth;       // 16 (RGB565) or 24 (RGB888)
    bool     supports_dimming;
} display_info_t;
```

## Drawing Primitives

```c
void display_fill_rect(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t color);
void display_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void display_draw_circle(int16_t cx, int16_t cy, uint16_t r, uint16_t color);
void display_draw_arc(int16_t cx, int16_t cy, uint16_t r, float start_deg, float end_deg, uint16_t color);
void display_draw_text(int16_t x, int16_t y, const char *text, const font_t *font, uint16_t color);
void display_flush(void);  // Transfer framebuffer to hardware
```

## Partial Update Support

For gauge applications, full-screen redraws are wasteful. The driver supports dirty-rectangle tracking:

```c
void display_mark_dirty(int16_t x, int16_t y, uint16_t w, uint16_t h);
void display_flush_dirty(void);  // Only transfers changed regions
```

## Files

```
display_driver/
├── README.md               # This file
├── CMakeLists.txt
├── include/
│   ├── display_driver.h     # Public API (hardware-agnostic)
│   ├── display_types.h      # Color, font, rect types
│   └── display_fonts.h      # Built-in font definitions
└── src/
    ├── display_driver.c      # Common driver logic, dirty tracking
    ├── ili9341.c             # ILI9341 SPI backend
    └── st7789.c              # ST7789 SPI backend
```

## Dependencies

- `spi_master` (ESP-IDF SPI driver)
- `driver/gpio` (backlight, DC, RST pins)
- `system` (logging)
