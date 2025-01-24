#include <nRF24L01.h>
#include <RF24.h>

#define NRF24_DATA_RATE          RF24_250KBPS // RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
#define NRF24_CHANNEL          100            // 0 ... 125
#define NRF24_PAYLOAD_SIZE      32            // Max. 32 bytes.
#define PROTOCOL 0x01                         // 0x01 (byte), temperature (float), humidity (float)
#define NRF24_PA_LEVEL           RF24_PA_MIN  // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX    



RF24 radio(7, 8); // CE, CSN

const byte address[6] = "1SNSR";
byte payload[32];             // Payload bytes. Used both for transmitting and receiving


int joystickPin = A1;  // Analog pin for joystick input
int joystickValue;
int pulseWidth;
int deadbandCenterMin = 950;  // Adjusted deadband minimum pulse width
int deadbandCenterMax = 1050;  // Adjusted deadband maximum pulse width
int neutralPulseWidth = 1000;  // Neutral (center position)
int minPulseWidth = 900;      // Minimum pulse width for ESC
int maxPulseWidth = 1100;      // Maximum pulse width for ESC


int steeringPin = A0;
int midPosition = 90;
int range = 25;
int servoAngle;


void setup() {
  Serial.begin(9600);
  radioConfig();

  pinMode(joystickPin, INPUT);
  pinMode(steeringPin, INPUT);
}

void loop() {
  int x = 10;
  int y = 10;

  joystickValue = analogRead(joystickPin);  // Read joystick input (0-1023)

  // Normalize joystick input to a range of -1 to 1
  float normalizedValue = (joystickValue - 512) / 512.0;

  // Implement deadband logic around the center (neutral)
  if (normalizedValue > -0.05 && normalizedValue < 0.05) {
    pulseWidth = neutralPulseWidth;  // Snap to neutral within the deadband
  } else {
    // Apply custom exponential scaling for smoother acceleration
    float exponentialValue = pow(2, abs(normalizedValue)) -1;  // Adjust curve: x^3 for smoother start
    if (normalizedValue < 0) {
      exponentialValue = -exponentialValue;  // Retain negative sign for reverse direction
    }

    // Map the exponential value (-1 to 1) to pulse width range
    pulseWidth = map(exponentialValue * 1000, 1000, -1000, minPulseWidth, maxPulseWidth);
  }

  int steeringValue = analogRead(steeringPin);
  float servoNormValue = steeringValue / 1023.0;
  float exponentialValue = pow(2, servoNormValue) - 1;  // input gradient applied for more precise control near the center
  servoAngle = map(exponentialValue * 1023, 0, 1023, midPosition + range, midPosition - range); // map servo angle based on input

  
  radio.stopListening();

  send(PROTOCOL, servoAngle, pulseWidth);
}

void send(byte protocol, int x, int y) {
  int offset = 0;  

  unsigned long timeMillis = millis();

  Serial.println("Preparing payload.");
  memcpy(payload + offset, (byte *)(&protocol), sizeof(protocol)); offset += sizeof(protocol); 
  memcpy(payload + offset, (byte *)(&timeMillis), sizeof(timeMillis)); offset += sizeof(timeMillis);
  memcpy(payload + offset, (byte *)(&x), sizeof(x)); offset += sizeof(x);
  memcpy(payload + offset, (byte *)(&y), sizeof(y)); offset += sizeof(y);

  Serial.print("Bytes packed: "); Serial.println(offset);

  if (radio.write(payload, offset)) {
    Serial.print("Payload sent successfully. Retries="); Serial.println(radio.getARC());
  }
  else {
    Serial.print("Failed to send payload. Retries="); Serial.println(radio.getARC());
  }   
}

void radioConfig(){
  radio.begin();
  while(!radio.begin()){
    Serial.println("error");
  }
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);
  radio.setPALevel(RF24_PA_MIN);

  radio.setDataRate(NRF24_DATA_RATE);
  radio.setChannel(NRF24_CHANNEL);
  radio.setPayloadSize(NRF24_PAYLOAD_SIZE);

  radio.setPALevel(NRF24_PA_LEVEL);
  
  radio.openWritingPipe(address);
  radio.stopListening();
}
