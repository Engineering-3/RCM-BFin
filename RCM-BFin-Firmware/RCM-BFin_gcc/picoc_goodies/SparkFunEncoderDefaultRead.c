/* A super simple script for reading a SparkFun 
 * QUWICC Twist RGB Rotary Encoder
 * DEV-15083
 * https://www.sparkfun.com/products/15083
 * This is for reading the encoder using the default
 * firmware shipped on the board on 05/24/2019.
 */

int i;
int count;
int I2CAddress = 0x40;

for (i=0; i < 200; i++)
{
  count = readi2c(I2CAddress, 0x06);
  count = (count << 8) | readi2c(I2CAddress, 0x05);
  if (count > 32767) 
  {
    count = -(65536 - count);
  }
  printf("Count = %d\n", count);
  delay(100);
}
exit(1);