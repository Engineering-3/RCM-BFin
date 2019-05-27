/* How to set and then read the Modified (v1.2)
 * SparkFun QUWICC Twist RGB Rotary Encoder
 * DEV-15083 (w/v1.2 firwmare)
 * https://www.sparkfun.com/products/15083
 */

int i;
int count;
int I2CAddress = 0x40;

// Read and print out the version number of the firmware
// make sure this is version 1.2
i = readi2c(I2CAddress, 0x02);
printf("version LSB = %d\n\r", i);
i = readi2c(I2CAddress, 0x03);
printf("version MSB = %d\n\r", i);

// (Optional) this 'resets' the encoder count to
// zero. So wherever the encoder is when this code
// runs becomes the new zero point
writei2c(I2CAddress, 0x05, 0x00);
writei2c(I2CAddress, 0x06, 0x00);

// Write 359 to Rotation Limit register (if your
// encoder has 360 counts/rotation)
writei2c(I2CAddress, 0x19, 0x67);   // LSB
writei2c(I2CAddress, 0x1A, 0x01);   // MSB

// Read back Rotation Limit register
i = readi2c(I2CAddress, 0x19);
printf("Rotation Limit LSB = %d\n\r", i);
i = readi2c(I2CAddress, 0x1A);
printf("Rotation Limit MSB = %d\n\r", i);

for (i=0; i < 200; i++)
{
  count = readi2c(I2CAddress, 0x06);
  count = (count << 8) | readi2c(I2CAddress, 0x05);
  printf("Count = %d\n", count);
  delay(100);
}
exit(1);
