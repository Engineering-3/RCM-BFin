/* Changes I2C address of 
 * SparkFun QUWICC Twist RGB Rotary Encoder
 * DEV-15083
 * https://www.sparkfun.com/products/15083
 */

int i;
int count;
int OldAddress = 0x3F;
int NewAddress = 0x41;

// Read the version number to make sure it's at the new address
i = readi2c(OldAddress, 0x02);
printf("Old Version MSB = %d\n\r", i);
i = readi2c(OldAddress, 0x03);
printf("Old Version LSB = %d\n\r", i);

// Write the new address
writei2c(OldAddress, 0x18, NewAddress);

// Read the version number to make sure it's at the new address
i = readi2c(NewAddress, 0x02);
printf("New Version MSB = %d\n\r", i);
i = readi2c(NewAddress, 0x03);
printf("New Version LSB = %d\n\r", i);

exit(1);
