/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "max31865.h"

/* MAX31865 on SPI1 CS pin 29 */
bfs::Max31865 rtd(&SPI1, 29);

int main() {
  /* Serial to display data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Init SPI and MAX31865 */
  SPI1.begin();
  if (!rtd.Begin(bfs::Max31865::RTD_3WIRE, 100.0f, 402.0f)) {
    Serial.println("ERROR initializing RTD");
    while (1) {}
  }
  while (1) {
    /* Read and print temperature */
    if (rtd.Read()) {
      Serial.println(rtd.temp_c());
      delay(50);
    }
  }
}
