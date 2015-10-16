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
float EEMEM EE_ADDRESS_0DBM_REFERENCE  = 0x0000;
float EEMEM EE_ADDRESS_50DBM_REFERENCE = 0x0080;
float EEMEM EE_ADDRESS_DBM_STEPSIZE  = 0x0040;
// für letzte PWM-Einstellung
uint8_t EEMEM EE_ADDRESS_PWM  = 0x00F0;

/* Aktuelle Referenzwerte */
float reference0dbm;
float reference50dbm;
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
uint16_t measuredValues[40];
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

double measuredPower = 0;
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

bool isMFTPushed(void) {
	// Bit 2 auf Port D
	return (PIND & (1<<2)) == 0;
}

void waitForMFTPush(void) {
	enableADConversion();
	while (!isMFTPushed()) {
		_delay_ms(25);
		enableADConversion();
	};
}

bool isUpPushed(void) {
	// Bit 3 auf Port D
	return (PIND & (1<<3)) == 0;
}

bool isDownPushed(void) {
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
void resetPins( void )
{
	PORTC &= ~(1 << LCD_RS);
	PORTC &= ~(1 << LCD_RW);
	PORTB &= ~0xFF;
}

/* Set the pins for LCD communication
* setPins(RS, RW, DB7 - DB4, DB3 - DB0);
* setPins(RW, RW,   DB7, DB6, DB5, DB4,   DB3, DB2, DB1, DB0);
*/
void setPins(bool _rs, bool _rw, bool d7, bool d6, bool d5, bool d4, bool d3, bool d2, bool d1, bool d0)
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
	// Data bits
	PORTB = (d7<<LCD_DB7) | (d6<<LCD_DB6) | (d5<<LCD_DB5) | (d4<<LCD_DB4) | (d3<<LCD_DB3) | (d2<<LCD_DB2) | (d1<<LCD_DB1) | (d0<<LCD_DB0);
}

/* Short enable to send bits to LCD */
void enable( void )
{
	PORTC |= (1 << LCD_EN);     // Enable auf 1 setzen
	_delay_us(20);  // kurze Pause
	PORTC &= ~(1 << LCD_EN);    // Enable auf 0 setzen
}

void printChar(int data) {
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

void printText(char text[256]) {
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
void clearLcd(void) {
	setPins(0, 0,   0, 0, 0, 0,   0, 0, 0, 1);
	enable();
	_delay_ms(2);
}

void initDisplay(void) {
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
	clearLcd();
	printText("Beleuchtung<einstellen:");
	_delay_ms(2000);
	// Beleuchtungsgrad in % anzeigen
	char tempArr[4];
	clearLcd();
	printText("Up/Down   MFT=OK<");
	sprintf(tempArr, "%d", (100*currentPWM/255));
	printText(tempArr);
	printText("%");
	
	while (isUpPushed()) ;
	// nachdem button up losgelassen wurde
	do {
		// PWM erhöhen
		if (isUpPushed() && !isDownPushed()) {
			if (currentPWM <= (0xE6)) {
				currentPWM += 25;
				OCR2 = currentPWM;
				} else {
				currentPWM = 0xFF;
				OCR2 = currentPWM;
			}
			// %-Anzeige aktualisieren
			clearLcd();
			printText("Up/Down   MFT=OK<");
			sprintf(tempArr, "%d", (100*currentPWM/255));
			printText(tempArr);
			printText("%");
			_delay_ms(400);
		}
		// PWM verringern
		if (isDownPushed() && !isUpPushed()) {
			if (currentPWM > 24) {
				currentPWM -= 25;
				OCR2 = currentPWM;
				} else {
				currentPWM = 0x00;
				OCR2 = currentPWM;
			}
			// %-Anzeige aktualisieren
			clearLcd();
			printText("Up/Down   MFT=OK<");
			sprintf(tempArr, "%d", (100*currentPWM/255));
			printText(tempArr);
			printText("%");
			_delay_ms(400);
		}
	} while (!isMFTPushed());
	clearLcd();
	printText("Beleuchtung<eingestellt!");
	eeprom_busy_wait();
	eeprom_write_byte(&EE_ADDRESS_PWM, currentPWM);
	_delay_ms(1500);
}

void calibrate(void) {
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
	clearLcd();
	printText("-50dBm anlegen<MFT drÜcken");
	/* Wait until button isn't pressed */
	
	waitForMFTPush();
	reference50dbm = (float) currentLogAmpSignal;
	eeprom_busy_wait();
	eeprom_write_float(&EE_ADDRESS_50DBM_REFERENCE, reference50dbm);
	stepSize = (reference0dbm - reference50dbm) / 50;
	eeprom_busy_wait();
	eeprom_write_float(&EE_ADDRESS_DBM_STEPSIZE, stepSize);

	clearLcd();
	printText("Kalibrierung<erfolgreich!");
	_delay_ms(1500);
}

void calculate(void) {
	if (previousLogAmpSignal == currentLogAmpSignal) {
		return;
	}
	measuredPower = (currentLogAmpSignal - reference0dbm) / stepSize;

	measuredMilliwatt = pow(10, ((measuredPower / 10) - 3)) * 1000;
	magneticFieldStrength = pow(10, (measuredPower + 60.0) / 20.0); // TODO Sondenkorrektur

	setUserFlag(F_NEW_DATA);
}

void printAfloat(double var) {
	uint32_t temp = var * 100;
	uint16_t a = var;
	uint16_t b = temp % 100;
	char before[10];
	char after[10];
	sprintf(before, "%d", a);
	sprintf(after, "%02d", b);
	uint8_t i = 0;
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
}

bool floorValue = true;
bool hugeChange = false;
#define TEXT_MAGNATIC_FIELD "0db:  "
#define FORMULA_N 1
#define FORMULA_A 0.01988
#define frequency 13560000
#define FORMULA_w (M_PI*2*frequency)
#define FORMULA_MueNull (M_PI*4 * 0.0000001)
#define FORMULA_MueR 1
void displayData(void) {
	if (!hasUserFlag(F_NEW_DATA) && !hasUserFlag(F_NEW_DATA)) {
		return;
	}
	double arrayAvrg = 0;
	signed int aValue;
	for(int j = 0; j < 40; j++) {
		arrayAvrg += measuredValues[j];
	}
	arrayAvrg /= 40;
	switch (currentOpMode) {
		case MODE_POWER:
			/* Arraywerte holen und Mittelwert bilden */
			
			aValue = arrayAvrg;
			/* Wenn Wert nicht geglättet werden soll wird currentLogAmpSignal genommen */
			if(!floorValue) {
				aValue = currentLogAmpSignal;
			}
			char str[6];
			sprintf(str, "%d", aValue);
			
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
			break;
		case MODE_MAGNETIC_FIELD:
			clearLcd();
			printText(TEXT_MAGNATIC_FIELD);
			double ergebnis = 2.56/1024 * (double)arrayAvrg;
			double dBm = (ergebnis - 1.996) / 0.0252; // = dBm
			/*             /--------------------------,         /--,
			 * Upeak = \  / 10^(dBm/10)  * 0.001 * 50 |  *  \  / 2 |
			 *eingang   \/                            |      \/    |
			 */
		    double Ue = sqrt(pow(10, (dBm/10)) * 0.001 * 50) * sqrt(2);
			/*               û
			 * H = ----------------------
			 *      N * A * w * µ0 * µr
			 */
			ergebnis = ((Ue) / (FORMULA_N * FORMULA_A * FORMULA_w * FORMULA_MueNull * FORMULA_MueR)) * 1000;
			printAfloat(reference0dbm);
			printText("<50db: ");
			//DBM
			printAfloat(reference50dbm);
			break;
	}
	unsetUserFlag(F_NEW_DATA);
	unsetUserFlag(F_OPMODE_CHANGE);
}

void voltageControl(void) {
	if (!hasUserFlag(F_HAS_WIRE) && currentVoltageBatt < (MIN_VOLTAGE_BATT + 10)) {
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
	// setADChannel(CHANNEL_VOLTAGE_BATT);

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
	reference50dbm = eeprom_read_float(&EE_ADDRESS_50DBM_REFERENCE);
	stepSize = eeprom_read_float(&EE_ADDRESS_DBM_STEPSIZE);

	setOpMode(MODE_POWER);
	setADChannel(CHANNEL_LOGAMP);
	
	/* Interrupts setup */
	MCUCR |= (1 << ISC01);
	MCUCR &= ~(1 << ISC00);	// interrupt on falling edge
	GICR = 1 << INT0;  //turn on Interrupt0
	MCUCR |= (1 << ISC11);
	MCUCR &= ~(1 << ISC10);	// interrupt on falling edge
	GICR = 1 << INT1;  //turn on Interrupt1
	sei();
	enableADConversion();

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
	for(int w = 0; w < 40; w++){
		_delay_ms(37);
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
	GICR |= 1<<INT0;
	GICR |= 1<<INT1;
	disableLogAmp();
	calculate();
	displayData();
	toggleLogAmp();
	while(1) {
		displayUpdateCounter++;
		/* Jeden 5. Loop (alle 250ms) display updaten */
		if(displayUpdateCounter == 10) {
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
			// disable interrupts while in settings
			GICR &= ~(1<<INT0);
			GICR &= ~(1<<INT1);
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
		// Button Interrupts wieder aktivieren wenn Down nicht mehr gedrückt
		if(hasUserFlag(F_IN_SETTINGS)) {
			if(!isDownPushed()) {
				GICR |= 1<<INT0;
				GICR |= 1<<INT1;
				unsetUserFlag(F_IN_SETTINGS);
			}
		}
		/* ADConversion alle 50ms */
		enableADConversion();
		_delay_ms(25);
	}
}

ISR(INT0_vect) {
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
}

ISR(INT1_vect) {
	if(!hasUserFlag(F_TOGGLE_LOG)) {
		setUserFlag(F_TOGGLE_LOG);
	}
	_delay_ms(1);
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
			if(arrayCounter > 39) {
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
