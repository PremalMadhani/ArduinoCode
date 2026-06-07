// the setup function runs once when you press reset or power the board

void setup() 
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(13, OUTPUT);
  pinMode(2, INPUT_PULLUP);

  Serial.begin(9600);
}

int i=0;
// the loop function runs over and over again forever
void loop() {
  if (digitalRead(2)==0)
  
  {
    Serial.println("Button is pressed");
    while (digitalRead(2)==0)
    {
      
      i++;
    }

    if (i%2 == 1)
    {
      digitalWrite(13,HIGH);
    }
    else
    {
      digitalWrite(13,LOW);
    }
  }
}
