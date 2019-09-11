/* An I2C scanner for PicoC
 * This code walks through every I2C address and prints out the ones that actually reply.
 */

int I2CAddress;
int value;

for (I2CAddress=0; I2CAddress < 0x80; I2CAddress++)
{
  value =  (readi2c(I2CAddress, 0x00));
  if (value > 0x00 && value < 0xFF)
  {
    printf("Found address 0x%02x replied with 0x%02x\n", (unsigned char)I2CAddress, (unsigned char)value);
  }
}
exit(1);