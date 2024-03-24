uint8_t count = 0 ;
uint8_t flag = 0;
void setup()
{
  pinMode(9, OUTPUT);
}

void loop()
{
  delay(10);
  if(count == 255||count == 0)
  {
	  flag = !flag;
  }
  if(1 == flag)
  {
    analogWrite(9,count++);
  }
  else
  {
    analogWrite(9,count--);
  }
}