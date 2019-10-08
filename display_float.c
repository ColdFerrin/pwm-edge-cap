//////////////////////////////////////////////////////////
//
char *floatToString(float x)
{
  // Use of static allocation here BAD code style
  // Functional but quick & dirty use only
  static char buf[32];
  int mantissa = (int)(x);
  int fraction = abs((int)((x - mantissa)*1000));
  if (fraction > 99)
  {
    sprintf(buf, "%d.%d    ", mantissa, fraction);
  } 
  else if (fraction > 9)
  {
    sprintf(buf, "%d.0%d    ", mantissa, fraction);
  }
  else
  {    
    sprintf(buf, "%d.00%d    ", mantissa, fraction);
  }
  
  return buf;
}