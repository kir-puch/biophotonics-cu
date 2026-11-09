
#include <Wire.h>
#include "MAX30105.h"
#include <Arduino_GFX_Library.h>
#include <pico/util/queue.h>
#include <array>
#include "heartRate.h" 

#define I2C_SDA 4 
#define I2C_SCL 5

#define TFT_CS   17
#define TFT_SCK  18
#define TFT_MOSI 19
#define TFT_DC   20
#define TFT_RST  21
#define TFT_BL   22

#define BUZZER_PIN   27


#define SPI_SPEED 75'000'000

// 
// SHARED
//

// for passing the samples from core1 to core0
struct Sample { uint32_t red, ir; };
queue_t sample_queue;
volatile bool queueReady = false;


//
// RENDERING
//

#define RGB565_IR RGB565(168, 0, 0)
#define HIST_DATA_POINTS 100

// 245 is the average ish if i just rest my finger on it
const uint32_t MIN_ACCEPTABLE_IR = 200'000;
const uint32_t MAX_ACCEPTABLE_IR = 300'000;

const uint32_t DISPLAY_Y_MIN = 80;
const uint32_t DISPLAY_Y_MAX = 310;
const uint32_t GRAPH_H = DISPLAY_Y_MAX - DISPLAY_Y_MIN;

Arduino_DataBus *bus = new Arduino_RPiPicoSPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, GFX_NOT_DEFINED, spi0);
Arduino_GFX *gfx = new Arduino_ST7796(bus, TFT_RST, 1 /* landscape */, true /* IPS */);
Arduino_Canvas *graph = new Arduino_Canvas(480, GRAPH_H, gfx, 0, DISPLAY_Y_MIN); // RAM copy of just the graph area (480 x 230 x 2 bytes = ~220 KB), shown at (0, DISPLAY_Y_MIN)

void setup() {
  queue_init(&sample_queue, sizeof(Sample), 64);   // room for ~0.6 s of samples
  queueReady = true;
  
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);   // backlight on

  gfx->begin(SPI_SPEED);
  graph->begin(GFX_SKIP_OUTPUT_BEGIN);
  gfx->fillScreen(RGB565_BLACK);
  gfx->setTextColor(RGB565_GREEN, RGB565_BLACK);
  gfx->setTextSize(2);
  gfx->setCursor(40, 20); // 40 140 is middle ish
  gfx->print("bfb: waiting");
}

float normalize_between(float value, float min_possible, float max_possible) {
  value -= min_possible;
  value /= (max_possible - min_possible);
  return constrain(value, 0, 1);
}
float denormalize_between(float value_between_0_and_1, float min_possible, float max_possible) {
  float delta = max_possible - min_possible;
  return value_between_0_and_1 * delta + min_possible;
}

// void clear_graph() {
//   graph->fillRect(
//     0, DISPLAY_Y_MIN, 
//     graph->width(), DISPLAY_Y_MAX - DISPLAY_Y_MIN, 
//     RGB565_BLACK
//   );
// }

void drawThickLine(Arduino_GFX *g, int x0, int y0, int x1, int y1, int t, uint16_t color) {
  for (int dx = -(t / 2); dx <= (t - 1) / 2; dx++)
    for (int dy = -(t / 2); dy <= (t - 1) / 2; dy++)
      g->drawLine(x0 + dx, y0 + dy, x1 + dx, y1 + dy, color);
}

void plot_graph(std::array<float, HIST_DATA_POINTS> &history, uint16_t color) {
  for (size_t x = 1; x < HIST_DATA_POINTS; ++x) {
    float y = history[x];
    y = normalize_between(y, MIN_ACCEPTABLE_IR, MAX_ACCEPTABLE_IR);
    y = denormalize_between(y, 0, GRAPH_H);

    float yPrev = history[x - 1];
    yPrev = normalize_between(yPrev, MIN_ACCEPTABLE_IR, MAX_ACCEPTABLE_IR);
    yPrev = denormalize_between(yPrev, 0, GRAPH_H);

    uint32_t xNorm = denormalize_between(
      normalize_between(x,     0, HIST_DATA_POINTS), 0, graph->width()
    );
    uint32_t xPrevNorm = denormalize_between(
      normalize_between(x - 1, 0, HIST_DATA_POINTS), 0, graph->width()
    );
    
    drawThickLine(
      graph, 
      xPrevNorm, yPrev, 
      xNorm, y, 
      2, color
    );
  }
}
void insert_array_shifting(std::array<float, HIST_DATA_POINTS> &history, size_t &history_filled, float value) {
  if (history_filled < HIST_DATA_POINTS) {
    history[history_filled] = value;
    history_filled += 1;
    return;
  }

  for (size_t i = 0; i < history_filled - 1; ++i) {
    history[i] = history[i + 1];
  }
  history[history.size() - 1] = value;
}

std::array<float, HIST_DATA_POINTS> ir_history;
size_t ir_history_filled;

std::array<float, HIST_DATA_POINTS> red_history;
size_t red_history_filled;

void loop() {
  Sample s = {0, 0};
  bool got_new_data = false;
  while (queue_try_remove(&sample_queue, &s)) {     // take everything that's waiting
    got_new_data = true;
    if (s.ir < MIN_ACCEPTABLE_IR || s.red < MIN_ACCEPTABLE_IR) {
      continue;
    }
    insert_array_shifting(ir_history, ir_history_filled, s.ir);
    insert_array_shifting(red_history, red_history_filled, s.red);
  }

  static uint32_t lastFrame = 0;
  if (millis() - lastFrame >= 40) {
    lastFrame = millis();

    graph->fillScreen(RGB565_BLACK);
    plot_graph(ir_history, RGB565_IR);
    plot_graph(red_history, RGB565_RED);
    graph->flush();
  }

  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate > 250) {
    lastUpdate = millis();
    
    if (s.ir < MIN_ACCEPTABLE_IR || s.red < MIN_ACCEPTABLE_IR) {
      gfx->setCursor(10, 20);
      gfx->setTextColor(RGB565_GREEN, RGB565_BLACK);
      if (!got_new_data) {
        gfx->print("<! sensor unresponsive !>     ");
      } else {
        gfx->print("<! no data !>                 ");
      }
    } else {
      gfx->setCursor(10, 20);
      gfx->setTextColor(RGB565_RED, RGB565_BLACK);
      gfx->printf("RED %8lu                     ", (unsigned long) s.red);
      gfx->setCursor(10, 20);
      gfx->setTextColor(RGB565_IR);
      gfx->printf("                 IR TEST %8lu     ", (unsigned long) s.ir);
    }
  }
}


//
// SENSORS
//

MAX30105 sensor;

void setup1() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  while (!queueReady) delay(1);

  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();

  while (!sensor.begin(Wire, I2C_SPEED_FAST))  {
    Serial.println("MAX30102 not found - check wiring and the 3.3V jumper");
    delay(1000);
  }

  // brightness 0-255, averaging, mode 2 = red+IR, sample rate, pulse width, ADC range
  sensor.setup(60, 4, 2, 100, 411, 4096);
  tone(BUZZER_PIN, 2000, 80);
}

void loop1() {
  sensor.check();
  while (sensor.available()) {
    uint32_t red = sensor.getFIFORed();
    uint32_t ir = sensor.getFIFOIR();
    Sample s = { red, ir };
    if (s.ir > MIN_ACCEPTABLE_IR && checkForBeat(s.ir)) {
      tone(BUZZER_PIN, 1500, 30);   // 1.5 kHz, 30 ms tick per heartbeat
    }
    queue_try_add(&sample_queue, &s);sensor
    

    sensor.nextSample();
  }
}
