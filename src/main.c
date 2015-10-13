#include <asf.h>
#include <avr/io.h>
#include <avr/interrupt.h>
/* CPU speed in Hz for delay (calculation) */
#define F_CPU 1000000UL
#include <util/delay.h>
#include <avr/eeprom.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Batteriespannung ist 9V. Bei etwa 6.5V ist die EIngangsspannung am Port PA2
 * etwa 1,48V (gegenüber der Referenzspannung 2.56V). Mit Umrechnung auf 10Bit
 * durch den AD-Wandler ergibt sich
 * 1.48V / 2.56V * 2^10 = 592 */
#define MIN_VOLTAGE_BATT 592
/* Bei 9V (minimaler Wert den die Netzspannung annehmen darf) ist
 * der Input 1.62V; gegenüber der Referenzspannung ergibt sich
 * 1.62V / 2.56V * 2^10 = 648 */
#define MIN_VOLTAGE_WIRE 648

/* PWM Konstante für Hell */
#define PWM_LIGHT 200

uint8_t currentPWM;

/* Korrekturwert für H-Sonde */
#define MAGNETIC_SONDE_CORR 60

/* Adressen im EEPROM */
// für Referenzwerte
float EE_ADDRESS_0DBM_REFERENCE EEMEM = 0x0000;
float EE_ADDRESS_DBM_STEPSIZE EEMEM = 0x0004;
// für letzte PWM-Einstellung
uint8_t EE_ADDRESS_PWM EEMEM = 0x00F0;

/* Aktuelle Referenzwerte */
float reference0dbm;
float stepSize;

/* Aktueller Messkanal
 * ADC0: LogAmp-Input
 * ADC1: Externe Spannung
 * ADC2: Batteriespannung
 */
uint8_t currentADChannel;
#define CHANNEL_LOGAMP 0
#define CHANNEL_VOLTAGE_WIRE 1
#define CHANNEL_VOLTAGE_BATT 2

/* ein array für mittelwertbildung */
uint16_t measuredValues[20];
uint8_t arrayCounter = 0;

/* Aktuelle Betriebsart
 * 0: Leistungsmessung
 * 1: H-Feld / H-Sonde
 * 2: E-Feld / E-Sonde // Not implemented yet
 * 3: E-Feld / H-Sonde // Not implemented yet
 */
uint8_t currentOpMode;
#define MODE_POWER 0
#define MODE_MAGNETIC_FIELD 1

/* Flags */
uint8_t userFlags = 0;
#define F_OPMODE_CHANGE 0
#define F_TOGGLE_LOG 1
#define F_NEW_DATA 2
#define F_HAS_WIRE 3
#define F_IN_SETTINGS 4

void setOpMode(uint8_t opMode) {
	currentOpMode = opMode;
	userFlags |= 1<<F_OPMODE_CHANGE;
}

uint8_t hasUserFlag(uint8_t flag) {
	return userFlags & (1<<flag);
}

void setUserFlag(uint8_t flag) {
	userFlags |= 1<<flag;
}

void unsetUserFlag(uint8_t flag) {
	userFlags &= ~(1<<flag);
}


/* Aktuelle Messwerte */
// Initialize to 10 (= -74dBm)
int currentLogAmpSignal = 10;
int previousLogAmpSignal = 10;
int currentVoltageWire = 0;
int currentVoltageBatt = 0;

signed int measuredPower = 0;
double measuredMilliwatt = 0;
double magneticFieldStrength = 0;

void enableADConversion(void) {
	// Setze ADSC (AD start conversion) im ADCSRA register
	ADCSRA |= 1<<ADSC;
}

void disableADConversion(void) {
	ADCSRA &= ~(1<<ADSC);
}

void enableLogAmp(void) {
	PORTD |= 1<<0;
}

void disableLogAmp(void) {
	PORTD &= ~(1<<0);
}

void toggleLogAmp(void) {
	PORTD ^= 1<<0;
}

void enableDisplayLight(void) {
	PORTD |= 1<<7;
}

void disableDisplayLight(void) {
	PORTD &= ~(1<<7);
}

void setADChannel(uint8_t channel) {
	if (channel < 0 || channel > 7) {
		// Nur 8 Kanäle sind vorhanden
		return;
		
	}
	// Alle bis auf erste drei Bit in ADMUX löschen
	ADMUX &= 0xE0;
	// Kanalnummer einschalten
	ADMUX |= channel;
	// Aktuellen Kanal merken
	currentADChannel = channel;
}

int isMFTPushed(void) {
	// Bit 2 auf Port D
	return (PIND & (1<<2)) == 0;
}

void waitForMFTPush(void) {
	while (!isMFTPushed()) ;
}

uint8_t isUpPushed(void) {
	// Bit 3 auf Port D
	return (PIND & (1<<3)) == 0;
}

uint8_t isDownPushed(void) {
	// Bit 4 auf Port D
	return (PIND & (1<<4)) == 0;
}

/***** LCD FUNCTIONS *****/
// LCD pins
#define LCD_RS		PC5
#define LCD_RW		PC6
#define LCD_EN		PC7
#define LCD_DB7		PB7
#define LCD_DB6		PB6
#define LCD_DB5		PB5
#define LCD_DB4		PB4
#define LCD_DB3		PB3
#define LCD_DB2		PB2
#define LCD_DB1		PB1
#define LCD_DB0		PB0

/* Reset all lcd-related pins */
static void resetPins( void )
{
	PORTC &= ~(1 << LCD_RS);
	PORTC &= ~(1 << LCD_RW);
	PORTB &= ~0xFF;
}

/* Set the pins for LCD communication
* setPins(RS, RW, DB7 - DB4, DB3 - DB0);
* setPins(RW, RW,   DB7, DB6, DB5, DB4,   DB3, DB2, DB1, DB0);
*/
static void setPins(bool _rs, bool _rw, bool d7, bool d6, bool d5, bool d4, bool d3, bool d2, bool d1, bool d0)
{
	// RS
	if (_rs) {
		PORTC |= (1 << LCD_RS);
	} else {
		PORTC &= ~(1 << LCD_RS);
	}
	// RW
	if (_rw)
	{
		PORTC |= (1 << LCD_RW);
		} else {
		PORTC &= ~(1 << LCD_RW);
	}
	// Data
	if (d7)
	{
		PORTB |= (1 << LCD_DB7);
	} else {
		PORTB &= ~(1 << LCD_DB7);
	}
	if (d6)
	{
		PORTB |= (1 << LCD_DB6);
	} else {
		PORTB &= ~(1 << LCD_DB6);
	}
	if (d5)
	{
		PORTB |= (1 << LCD_DB5);
	} else {
		PORTB &= ~(1 << LCD_DB5);
	}
	if (d4)
	{
		PORTB |= (1 << LCD_DB4);
	} else {
		PORTB &= ~(1 << LCD_DB4);
	}
	if (d3)
	{
		PORTB |= (1 << LCD_DB3);
	} else {
		PORTB &= ~(1 << LCD_DB3);
	}
	if (d2)
	{
		PORTB |= (1 << LCD_DB2);
		} else {
		PORTB &= ~(1 << LCD_DB2);
	}
	if (d1)
	{
		PORTB |= (1 << LCD_DB1);
	} else {
		PORTB &= ~(1 << LCD_DB1);
	}
	if (d0)
	{
		PORTB |= (1 << LCD_DB0);
	} else {
		PORTB &= ~(1 << LCD_DB0);
	}
}

/* Short enable to send bits to LCD */
static void enable( void )
{
	PORTC |= (1 << LCD_EN);     // Enable auf 1 setzen
	_delay_us(20);  // kurze Pause
	PORTC &= ~(1 << LCD_EN);    // Enable auf 0 setzen
}

static void printChar(int data) {
	/* Set pins to send data */
	PORTC |= (1 << LCD_RS);
	PORTC &= ~(1 << LCD_RW);
	/*
	* Apply the data to port B
	* char = int = hex (ASCII)
	* so no conversion needed.
	*/
	PORTB = data;
	/* Sending */
	enable();
	_delay_us(50);
}

static void printText(char text[256]) {
	int i = 0;
	/*
	* Loop through array and print
	* the characters.
	*/
	do 
	{
		// < sign to determine a new line, since \ is reserved
		if(text[i] == '<') {
			setPins(0, 0,   1, 1, 0, 0,   0, 0, 0, 0);
			enable();
			_delay_us(60);
		} else {
			printChar(text[i]);
		}
		i++;
	} while (text[i] != 0);
}

/* Clearing the LCD */
static void clearLcd(void) {
	setPins(0, 0,   0, 0, 0, 0,   0, 0, 0, 1);
	enable();
	_delay_ms(2);
}

static void initDisplay(void) {
	resetPins();
	/* Display initialization sequence */
	setPins(0, 0,   0, 0, 1, 1,   0, 0, 0, 0);
	enable();
	_delay_ms(5);
	enable();
	_delay_ms(1);
	enable();
	_delay_ms(2);
	/* Sended 3 times */
	resetPins();
	/* 8-bit interface, 2-line display, 5x7 dots */
	setPins(0, 0,   0, 0, 1, 1,   1, 0, 0, 0);
	enable();
	_delay_us(50);
	/* Display on, cursor on, blinking on */
	setPins(0, 0,   0, 0, 0, 0,   1, 1, 1, 1);
	enable();
	_delay_us(50);
	/* Clear display */
	setPins(0, 0,   0, 0, 0, 0,   0, 0, 0, 1);
	enable();
	_delay_ms(2);
	/* Entry Mode Set */
	/* Cursor move, display no shift */
	setPins(0, 0,   0, 0, 0, 0,   0, 1, 1, 0);
	enable();
	_delay_ms(50);
}

/***** Other functions *****/
void setDisplayLight(void) {
	// TODO display 'Beleuchtung einstellen'
	clearLcd();
	printText("Beleuchtung<einstellen:");
	_delay_ms(2000);
	char tempArr[4];
	clearLcd();
	printText("Up/Down   MFT=OK<");
	sprintf(tempArr, "%d", (100*currentPWM/255));
	printText(tempArr);
	printText("%");
	
	while (isUpPushed()) ;
	do {
		if (isUpPushed() && !isDownPushed()) {
			if (currentPWM <= (0xE6)) {
				currentPWM += 25;
				OCR2 = currentPWM;
				} else {
				currentPWM = 0xFF;
				OCR2 = currentPWM;
			}
			setPins(0, 0,   0, 1, 1, 0,   0, 0, 0, 0);
			enable();
			_delay_us(50);
			clearLcd();
			printText("Up/Down   MFT=OK<");
			sprintf(tempArr, "%d", (100*currentPWM/255));
			printText(tempArr);
			printText("%");
			_delay_ms(400);
		}
		if (isDownPushed() && !isUpPushed()) {
			if (currentPWM > 24) {
				currentPWM -= 25;
				OCR2 = currentPWM;
				} else {
				currentPWM = 0x00;
				OCR2 = currentPWM;
			}
			setPins(0, 0,   0, 1, 1, 0,   0, 0, 0, 0);
			enable();
			_delay_us(50);
			clearLcd();
			printText("Up/Down   MFT=OK<");
			sprintf(tempArr, "%d", (100*currentPWM/255));
			printText(tempArr);
			printText("%");
			_delay_ms(400);
		}
	} while (!isMFTPushed());
	// TODO display 'Beleuchtung eingestellt'
	clearLcd();
	printText("Beleuchtung<eingestellt!");
	eeprom_busy_wait();
	eeprom_write_byte(&EE_ADDRESS_PWM, currentPWM);
	_delay_ms(1500); // evtl zu kurz?
	// TODO Tasten-Flags löschen?
}

void calibrate(void) {
	// TODO display '0dBm anlegen\n MFT drücken'
	clearLcd();
	printText("-0.1dBm anlegen<MFT drÜcken");
	
	waitForMFTPush();
	reference0dbm = (float) currentLogAmpSignal;
	eeprom_busy_wait();
	eeprom_write_float(&EE_ADDRESS_0DBM_REFERENCE, reference0dbm);
	_delay_ms(250);
	while (isMFTPushed()) {
		_delay_ms(200);
	}
	// TODO display '-50dBm anlegen\n MFT drücken'
	clearLcd();
	printText("-50dBm anlegen<MFT drÜcken");
	/* Wait until button isn't pressed */
	
	waitForMFTPush();
	float dbm50Signal = (float) currentLogAmpSignal;
	stepSize = (reference0dbm - dbm50Signal) / 50;
	eeprom_busy_wait();
	eeprom_write_float(&EE_ADDRESS_DBM_STEPSIZE, stepSize);

	// TODO display 'kalibriert'
	clearLcd();
	printText("Kalibrierung<erfolgreich!");
	_delay_ms(1500); // evtl zu kurz 500 -> 1500?
}

void calculate(void) {
	if (previousLogAmpSignal == currentLogAmpSignal) {
		return;
	}
	measuredPower = (currentLogAmpSignal - reference0dbm) / stepSize;

	measuredMilliwatt = pow(10.0, ((measuredPower / 10) - 3)) * 1000;
	magneticFieldStrength = pow(10.0, (measuredPower + MAGNETIC_SONDE_CORR) / 20); // TODO Sondenkorrektur

	setUserFlag(F_NEW_DATA);
}

bool floorValue = true;
bool hugeChange = false;
#define TEXT_MAGNATIC_FIELD "LogAmp out:<"
void displayData(void) {
	if (!hasUserFlag(F_NEW_DATA) && !hasUserFlag(F_NEW_DATA)) {
		return;
	}
	switch (currentOpMode) {
		case MODE_POWER:
			/* Arraywerte holen und Mittelwert bilden */
			for(int j = 0; j < 20; j++) {
				measuredPower += measuredValues[j];
			}
			measuredPower /= 20;
			/* Wenn Wert nicht geglättet werden soll wird currentLogAmpSignal genommen */
			if(!floorValue) {
				measuredPower = currentLogAmpSignal;
			}
			char str[6];
			sprintf(str, "%d", measuredPower);
			
			// TODO display '             dBm'
			/* Set cursor line 1,1 */
			clearLcd();
			int i = 0;
			do
			{
				printChar(str[i]);
				i++;
			} while (str[i] != 0);
			printText(" ADC LogAmp");
			if(!floorValue) {
				printText("<nicht ø");
			} else if(hugeChange) {
				printText("<rechne...");
				hugeChange = false;
			}
			// TODO display Messwert einfügen: measuredPower
			// TODO display (Zeile 2): Berechnete Milliwatt: measuredMilliwatt
			break;
		case MODE_MAGNETIC_FIELD:
			magneticFieldStrength = 0;
			// TODO display 'H-Feld / H-Sonde'
			clearLcd();
			printText(TEXT_MAGNATIC_FIELD);
			double testF = 2.56/1024 * (double)currentLogAmpSignal;
			uint32_t temp = testF * 100;
			uint16_t a = testF;
			uint16_t b = temp % 100;
			char before[4];
			char after[2];
			sprintf(before, "%d", a);
			sprintf(after, "%d", b);
			if(b < 10) {
				char t = after[0];
				after[0] = '0';
				after[1] = t;
			}
			// TODO display (Zeile 2) 'mA/m = magneticFieldStrength'
			i = 0;
			do
			{
				printChar(before[i]);
				i++;
			} while (before[i] != NULL);
			i=0;
			if(after != NULL) {
				printChar('.');
			}
			do
			{
				printChar(after[i]);
				i++;
			} while (after[i] != NULL);
			printText(" V");
			break;
	}
	unsetUserFlag(F_NEW_DATA);
	unsetUserFlag(F_OPMODE_CHANGE);
}

void voltageControl(void) {
	if (!hasUserFlag(F_HAS_WIRE) && currentVoltageBatt < (MIN_VOLTAGE_BATT + 10)) {
		// TODO display 'Batterie schwach'
		clearLcd();
		printText("Batterie schwach");
		do {
			_delay_ms(1000);
		} while (currentVoltageBatt < MIN_VOLTAGE_BATT);
	}

	if (currentVoltageWire > MIN_VOLTAGE_WIRE) {
		setUserFlag(F_HAS_WIRE);
	} else {
		unsetUserFlag(F_HAS_WIRE);
	}
}

int main(int argc, const char *argv[])
{
	
	// Initialization
	/* Alternative Funktionen der Pins abschalten 
	 * damit sie als normale I/O verwendet werden können
	 */
	MCUCSR |= (1 << JTD);
	MCUCSR |= (1 << JTD); // yep, needed twice!
	SPCR &= ~(1 << SPE);
	// PORTS
	
	/* Ports:
	 * DDR  -> data direction register (0 input, 1 output)
	 * PORT -> Bei input: Pull-up-Widerstand ja/nein; bei output: Signal
	 * PIN  -> Input-Signal
	 */
	/* A: input */
	DDRA  = 0x00;
	PORTA = 0x00;
	/* B: display output */
	DDRB  = 0xFF;
	/* C5-7: display RS, R/W, E */
	DDRC  = 0xE0; // 1110 0000
	PORTC = 0x1F; // 0001 1111
	/* D0: enable LogAmp. D7: PWM */
	DDRD  = 0x81; // 1000 0001 -> |= (1<<PD0)
	PORTD |= (1<<PD0);
	/* D2-4 buttons als inputs
	 * Data Direction (DDRD) auf 0
	 * Pins auf 1 pullen, da button auf 0 zieht
	 */ 
	DDRD &= ~(1 << PD2);
	PORTD |= (1 << PD2);
	DDRD &= ~(1 << PD3);
	PORTD |= (1 << PD3);
	DDRD &= ~(1 << PD4);
	PORTD |= (1 << PD4);
	
	/* 2.56V Spannungsreferenz: 1100 0000 (REFS0 + REFS1)
	 * Byte-Reihenfolge für 10-bit-Ausgabe: 00x0 0000 (ADLAR auf 0)
	 * MUX0-4 (000x xxxx) auf 0 für unverstärkten ADC0 Eingang (MUX4-0)
	 * (s. Funktion changeADChannel(int channel))
	 * => 1100 0000 = 0xE0
	 */
	ADMUX = 0xC0;

	/* Prescaler auf 128: 0000 0111
	 * AD conversion aktivieren: 1000 0000 (ADEN)
	 * AD conversion interrupt einschalten: 0000 1000 (ADIE)
	 * => 1000 1111 = 0x8F
	 */
	ADCSRA = 0x8F;

	// Starten mit Batterieprüfung
	setADChannel(CHANNEL_VOLTAGE_BATT);

	// PWM-Initialisierung
	/* Timer/Counter Control Register 2
	 * Bit 7: Force Output Compare: no
	 * Bit 3,6: Fast PWM mode => 1,1
	 * Bit 5,4: Clear OC2 on compare match => 1,0
	 * Bit 2,1,0: Clock select: clk/64 => 1,0,0
	 * => 01101100 = 0x6C
	 */
	TCCR2 = 0x6C;
	eeprom_busy_wait();
	currentPWM = eeprom_read_byte(&EE_ADDRESS_PWM);
	/* OCR2 = Obergrenze für PWM */
	OCR2 = currentPWM;
	
	// Initialisierung der ADC-Referenzwerte
	reference0dbm = eeprom_read_float(&EE_ADDRESS_0DBM_REFERENCE);
	stepSize = eeprom_read_float(&EE_ADDRESS_DBM_STEPSIZE);

	setOpMode(MODE_POWER);
	setADChannel(CHANNEL_LOGAMP);
	
	/* Interrupts setup */
	GICR = 1 << INT1;  //turn on Interrupt1
	MCUCR = (1 << ISC11) & ~(1 << ISC10);	// interrupt on falling edge
	GICR = 1 << INT0;  //turn on Interrupt0
	MCUCR = (1 << ISC01) & ~(1 << ISC00);	// interrupt on falling edge
	sei();
	enableADConversion();

	// TODO Initialisierung des Displays
	/* Display initialization */
	initDisplay();
	uint8_t displayUpdateCounter = 0;
	/*
	H-Feld-Messgerät
	      V1.0
	*/
	printText("H-Feld-Messger");
	printChar(0xE1);
	printText("t<      V2.0");
	for(int w = 0; w < 20; w++){
		_delay_ms(75);
		enableADConversion();
	}
	clearLcd();
	/*if (currentVoltageBatt < (MIN_VOLTAGE_BATT + 20)) {
		// TODO display 'Batterie schwach'
		clearLcd();
		printText("Batterie schwach");
		do {
			_delay_ms(1000);
		} while ((currentVoltageBatt < (MIN_VOLTAGE_BATT)+20) || (currentVoltageWire > MIN_VOLTAGE_WIRE));
	}*/

	if (isMFTPushed()) {
		while (isMFTPushed()) {
			_delay_ms(300);
			printText("Kalibrierung:<Taste loslassen!");
		}
		calibrate();
	}

	if (isUpPushed()) {
		setDisplayLight();
	}

	/* Interrupts for INT0 und INT1 aktivieren */
	GICR |= 1<<6;
	GICR |= 1<<7;
	disableLogAmp();
	calculate();
	displayData();
	toggleLogAmp();
	while(1) {
		displayUpdateCounter++;
		/* Jeden 5. Loop (alle 250ms) display updaten */
		if(displayUpdateCounter == 5) {
			/* Zu Testzwecken
			 * Wenn Button UP Flag F_TOGGLE_LOG gesetzt hat
			 * wird die Wertglättung an/aus geschaltet.
			 * Zum besseren Entprellen auch nur alle 250ms verarbeiten.
			 */
			if(hasUserFlag(F_TOGGLE_LOG)) {
				//toggleLogAmp();
				floorValue = !floorValue;
				unsetUserFlag(F_TOGGLE_LOG);
			}
			displayUpdateCounter = 0;
			calculate();
			displayData();
		}
		// TODO display
		// TODO Betriebsmodus, Helligkeit etc. schalten
		//voltageControl();
		if (currentOpMode != MODE_POWER && isDownPushed()) {
			// TODO
			//calibrateSonde();
		}
		/* Down button aktiviert minimalen SETTINGS Modus
		 * Down + MFT -> kalibrieren
		 * Down + Up -> Helligkeit einstellen
		 */
		if(isDownPushed()) {
			setUserFlag(F_IN_SETTINGS);
			if(isUpPushed()) {
				setDisplayLight();
			} else if (isMFTPushed()) {
				while (isMFTPushed()) {
					_delay_ms(300);
					clearLcd();
					printText("Kalibrierung:<Taste loslassen!");
				}
				calibrate();
			}
		}
		/* Flag clearen wenn down nicht mehr gedrückt ist */
		if(hasUserFlag(F_IN_SETTINGS)) {
			if(!isDownPushed()) {
				unsetUserFlag(F_IN_SETTINGS);
			}
		}
		/* ADConversion alle 50ms */
		enableADConversion();
		_delay_ms(50);
	}
}

ISR(INT0_vect) {
	if(!hasUserFlag(F_IN_SETTINGS)) {
		if (!hasUserFlag(F_OPMODE_CHANGE)) {
			switch (currentOpMode) {
				case MODE_POWER:
					setOpMode(MODE_MAGNETIC_FIELD);
					break;
				case MODE_MAGNETIC_FIELD:
					setOpMode(MODE_POWER);
					break;
			}
		}
		_delay_ms(1);
		// TODO Warten bis Taste losgelassen ist?
	}
}

ISR(INT1_vect) {
	/*if(!hasUserFlag(F_IN_SETTINGS)) {
		setUserFlag(F_TOGGLE_LOG);
		if (currentOpMode == MODE_POWER) {
			// do what?
		} else {
			if (!isUpPushed()) {
				// do what?
			}
		}
		_delay_ms(1);
		// TODO Warten bis Taste losgelassen ist?
	}*/
	setUserFlag(F_TOGGLE_LOG);
}

ISR(ADC_vect) {
	uint8_t low = ADCL;
	uint8_t high = ADCH;
	int result = (high << 8) + low;
	switch(currentADChannel) {
		case CHANNEL_LOGAMP:
			// Ergebnis glätten
			currentLogAmpSignal += result;
			currentLogAmpSignal /= 2;
			/* Durchs array rotieren und Werte erneuern */
			if(arrayCounter > 19) {
				arrayCounter = 0;
			}
			/* Bei großer Änderung hugeChange auf true setzen */
			if((measuredValues[arrayCounter] < currentLogAmpSignal-40) || (measuredValues[arrayCounter] > currentLogAmpSignal+40)) {
				hugeChange = true;
			}
			/* Wert schreiben */
			measuredValues[arrayCounter] = currentLogAmpSignal;
			arrayCounter++;
		/*	setADChannel(CHANNEL_VOLTAGE_BATT);
			break;
		case CHANNEL_VOLTAGE_BATT:
			currentVoltageBatt = result;
			setADChannel(CHANNEL_LOGAMP);
			break;*/
	}
}
