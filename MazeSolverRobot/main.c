/*
 * MazeSolverRobot.c
 *
 * Created: 8/13/2017 2:30:44 PM
 * Author : Vost
 */ 

#define F_CPU 1000000UL
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )
#define LEFT 0
#define FORWARD 1
#define RIGHT 2
#define GOAHEAD 0
#define TURN 1
#define HIGH 0x01
#define LOW 0x00
#define ECHO 0x01//Pin PB0

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void sensorMotor(uint8_t);
void setTrigger(void);
double pulseIn(uint8_t, uint8_t);
double getDistance(uint8_t, uint8_t);
void dirMotor(uint8_t);

int main(void){
	double distance = 0, wallDistance = 0;
	//Configuracion de Entradas y salidas
	DDRB = 0xFE;
	DDRD = 0xFF;
	//Configuracion de Timer0 - PWM para el motor de torque
	OCR0A = 255;
	TCCR0A = 0x23; //Modo no invertido
	TCCR0B = 0x09; //Reloj sin pre-escaler
	//Configuracion de Timer1 - PWM para los servomotores
	ICR1 = 19999; //Coloca el Maximo
	TCCR1A = 0xA2; //PWM rápido no invertido en Modo 14
	TCCR1B = 0x19; //Reloj sin pre-escalador
	sei();	
	OCR0B = 250;
	PORTD |= (1 << PORTD1); // PD1 goes high
	PORTD &= ~(1 << PORTD2); // PD2 goes low
	while (1){
		sensorMotor(RIGHT);//Mano derecha en la pared
		_delay_ms(280);
		wallDistance = getDistance(ECHO, HIGH);
		if (wallDistance > 8){
			dirMotor(RIGHT);
			while (wallDistance  > 15){
				_delay_ms(10);
				wallDistance = getDistance(ECHO, HIGH);
			}
			dirMotor(FORWARD);
		}
		sensorMotor(FORWARD);
		_delay_ms(280);
		distance = getDistance(ECHO, HIGH);
		if (distance < 20){
			dirMotor(LEFT);
			while (distance < 50){
				_delay_ms(10);
				distance = getDistance(ECHO, HIGH);
			}
			dirMotor(FORWARD);
		}
	}
}

double getDistance(uint8_t pin, uint8_t state){
	double pulse, distance;
	setTrigger();
	pulse = pulseIn(pin, state);
	distance = pulse * 0.017;
	return distance;
}

/* Measures the length (in microseconds) of a pulse on the pin; state is HIGH
 * or LOW, the type of pulse to measure.  Works on pulses from 2-3 microseconds
 * to 3 minutes in length, but must be called at least a few dozen microseconds
 * before the start of the pulse. */
double pulseIn(uint8_t bit, uint8_t state){
	uint8_t stateMask = (state ? bit : 0);
	uint16_t width = 0; // keep initialization out of time critical area

	// convert the timeout from microseconds to a number of times through
	// the initial loop; it takes 16 clock cycles per iteration.
	unsigned long numloops = 0;
	unsigned long maxloops = microsecondsToClockCycles(180000000) / 16; //3 min = 180 s = 180000000 uS
	
	// wait for the pulse to start
	while ((PINB & bit) != stateMask){//Escanea el puerto B y compara el estado del bit 
		if (numloops++ == maxloops)
			return 0;
	}
	
	// wait for the pulse to stop
	while ((PINB & bit)  == stateMask) {//Escanea el puerto B y compara el estado del bit 
		if (numloops++ == maxloops)
			return 0;
		width++;
	}
						
	// convert the reading to microseconds. The loop has been determined
	// to be 20 clock cycles long and have about 16 clocks between the edge
	// and the start of the loop. There will be some error introduced by
	// the interrupt handlers.
	return clockCyclesToMicroseconds(width * 17 + 16); 
}

//Generamos el disparo
void setTrigger(void){
	_delay_us(2);
	PORTD |= (1 << PORTD0); // PD0 goes high
	_delay_us(10);
	PORTD &= ~(1 << PORTD0); // PD0 goes low
}

void sensorMotor(uint8_t pos){
	if (pos == LEFT){
		OCR1B = 2180; //90 grados 2120
	}else if (pos == FORWARD){
		OCR1B = 1370; //0 grados 1490
	}else if (pos == RIGHT){
		OCR1B = 535; //-90 grados
	}
}

void dirMotor(uint8_t pos){
	if (pos == LEFT){
		OCR1A = 465; //-90 grados
	}else if (pos == FORWARD){
		OCR1A = 1535; //0 grados
	}else if (pos == RIGHT){
		OCR1A = 2200; //90 grados 2265
	}
}
