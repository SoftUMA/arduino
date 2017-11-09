//--------------------------------------------------------------------------
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <cctype>
#include <cmath>
using namespace std;
//--------------------------------------------------------------------------
namespace user__ {
	typedef bool boolean;
	typedef unsigned char byte;
	typedef unsigned short word;
	//--------------------------------
	static const byte INPUT = 0;
	static const byte INPUT_PULLUP = 0;
	static const byte OUTPUT = 1;
	static const byte LOW = 0;
	static const byte HIGH = 1;
	static const byte A0 = 0;
	static const byte A1 = 1;
	static const byte A2 = 2;
	static const byte A3 = 3;
	static const byte A4 = 4;
	static const byte A5 = 5;
	static const byte DEC = 0;
	static const byte HEX = 1;
	static const byte BIN = 2;
	//--------------------------------
	extern const char* SALIDA;
	//------------------------------------
	void setup();
	void loop();
}
//--------------------------------------------------------------------------
namespace simulacion__ {
	//--------------------------------
	using user__::boolean;
	using user__::byte;
	using user__::word;
	//--------------------------------
	using user__::INPUT;
	using user__::INPUT_PULLUP;
	using user__::OUTPUT;
	using user__::LOW;
	using user__::HIGH;
	using user__::A0;
	using user__::A1;
	using user__::A2;
	using user__::A3;
	using user__::A4;
	using user__::A5;
	using user__::DEC;
	using user__::HEX;
	using user__::BIN;
	//--------------------------------
	using user__::SALIDA;
	//--------------------------------
	static void simulacion_setup();
	static void simulacion_loop();
}
//--------------------------------------------------------------------------
namespace internal__ {
	//--------------------------------
	using user__::boolean;
	using user__::byte;
	using user__::word;
	//--------------------------------
	using user__::INPUT;
	using user__::INPUT_PULLUP;
	using user__::OUTPUT;
	using user__::LOW;
	using user__::HIGH;
	using user__::A0;
	using user__::A1;
	using user__::A2;
	using user__::A3;
	using user__::A4;
	using user__::A5;
	using user__::DEC;
	using user__::HEX;
	using user__::BIN;
	//--------------------------------
	using user__::SALIDA;
	//----------------------------------------------------------------------
	//-- String ------------------------------------------------------------
	//----------------------------------------------------------------------
	class String {
		std::string s;
	public:
		String() {}
		String(const char* x) : s(x) {}
		String(char x) : s() {
			char xx[8];
			sprintf(xx, "%c", x);
			s = xx;
		}
		String(signed char x) : s() {
			char xx[8];
			sprintf(xx, "%c", x);
			s = xx;
		}
		String(unsigned char x) : s() {
			char xx[8];
			sprintf(xx, "%d", (int)x);
			s = xx;
		}
		String(long int x) : s() {
			char xx[64];
			sprintf(xx, "%ld", x);
			s = xx;
		}
		String(long int x, int base) : s() {
			char xx[64];
			switch (base) {
				case BIN:
				case HEX: sprintf(xx, "%lX", x); break;
				default: sprintf(xx, "%ld", x); break;
			}
			s = xx;
		}
		String(unsigned long x) : s() {
			char xx[64];
			sprintf(xx, "%lu", x);
			s = xx;
		}
		String(unsigned long x, int base) : s() {
			char xx[64];
			switch (base) {
				case BIN:
				case HEX: sprintf(xx, "%lX", x); break;
				default: sprintf(xx, "%lu", x); break;
			}
			s = xx;
		}
		String(double x) : s() {
			char xx[64];
			sprintf(xx, "%g", x);
			s = xx;
		}
		String(double x, int dec) : s() {
			char xx[64];
			sprintf(xx, "%.*g", dec, x);
			s = xx;
		}
		String(const String& o) : s(o.s) {}
		String& operator=(const String& o) { s = o.s; return *this; }
		bool concat(const String& x) { s += x.s; return true; }
		String& operator+=(const String& x) { s += x.s; return *this; }
		friend String operator+(const String& x, const String& y) {
			String r(x);
			r += y;
			return r;
		}
		std::string get() const { return s; }
	};
	//----------------------------------------------------------------------
	//-- Serial ------------------------------------------------------------
	//----------------------------------------------------------------------
	class Serial_ {
		ofstream file;
	public:
		Serial_() {
			file.open(SALIDA);
		}
		~Serial_() {
			if (file) {
				file.close();
			}
		}
		operator bool() const { return ! file.fail(); }
		void begin(int) const {}
		void flush() { file.flush(); }
		void print(char x) { file << x; }
		void print(signed char x) { file << x; }
		void print(unsigned char x) { file << (unsigned int)x; }
		void print(short x) { file << x; }
		void print(unsigned short x) { file << x; }
		void print(int x) { file << x; }
		void print(unsigned int x) { file << x; }
		void print(long x) { file << x; }
		void print(unsigned long x) { file << x; }
		void print(float x) { file << x; }
		void print(double x) { file << x; }
		void print(const char* x) { file << x; }
		void print(const std::string& val) { file << val; }
		void print(const String& val) { file << val.get(); }
		void println() { file << std::endl; }
		void println(char x) { file << x << std::endl; }
		void println(signed char x) { file << x << std::endl; }
		void println(unsigned char x) { file << (unsigned int)x << std::endl; }
		void println(short x) { file << x << std::endl; }
		void println(unsigned short x) { file << x << std::endl; }
		void println(int x) { file << x << std::endl; }
		void println(unsigned int x) { file << x << std::endl; }
		void println(long x) { file << x << std::endl; }
		void println(unsigned long x) { file << x << std::endl; }
		void println(float x) { file << x << std::endl; }
		void println(double x) { file << x << std::endl; }
		void println(const char* x) { file << x << std::endl; }
		void println(const std::string& val) { file << val << std::endl; }
		void println(const String& val) { file << val.get() << std::endl; }
	};
	static Serial_ Serial;
	//----------------------------------------------------------------------
	//-- Arduino -----------------------------------------------------------
	//----------------------------------------------------------------------
	class Arduino_ {
		unsigned long tiempo;
		int analogico[6];
		int digital[14];
		int modo[14];
	public:
		Arduino_() : tiempo(0) {
			for (int i = 0; i < 6; ++i) {
				analogico[i] = 0;
			}
			for (int i = 0; i < 14; ++i) {
				digital[i] = LOW;
				modo[i] = INPUT;
			}
		}
		unsigned long millis() const {
			return tiempo;
		}
		void inc_tiempo() {
			++tiempo;
		}
		void delay(unsigned long ms) {
			unsigned long fin = tiempo + ms;
			while (tiempo < fin) {
				inc_tiempo();
				simulacion__::simulacion_loop();
			}
		}
		void pinMode(int p, int m) {
			if ((0 <= p && p < 14)&&(m == INPUT || m == OUTPUT)) {
				modo[p] = m;
			} else {
				Serial.print("#> ERROR: pinMode(");
				Serial.print(p);
				Serial.print(", ");
				Serial.print(m);
				Serial.println(")");
			}
		}
		void digitalWrite(int p, int v) {
			if ((0 <= p && p < 14)&&(v == LOW || v == HIGH)&&(modo[p] == OUTPUT)) {
				digital[p] = v;
			} else {
				Serial.print("#> ERROR: digitalWrite(");
				Serial.print(p);
				Serial.print(", ");
				Serial.print(v);
				Serial.println(")");
			}
		}
		int digitalRead(int p) const {
			int v = 0;
			if ((0 <= p && p < 14)&&(modo[p] == INPUT)) {
				v = digital[p];
			} else {
				Serial.print("#> ERROR: digitalRead(");
				Serial.print(p);
				Serial.println(")");
			}
			return v;
		}
		void analogWrite(int p, int v) {
			if ((p == 3 || p == 5 || p == 6 || p == 9 || p == 10 || p == 11)
				&&(0 <= v && v <= 255)&&(modo[p] == OUTPUT)) {
				digital[p] = v;
		} else {
			Serial.print("#> ERROR: analogWrite(");
			Serial.print(p);
			Serial.print(", ");
			Serial.print(v);
			Serial.println(")");
		}
	}
	int analogRead(int p) const {
		int v = 0;
		if (0 <= p && p < 6) {
			v = analogico[p];
		} else {
			Serial.print("#> ERROR: analogRead(");
			Serial.print(p);
			Serial.println(")");
		}
		return v;
	}
		//--------------------------------
	int getDigitalVal(int p) const {
		int v = 0;
		if ((0 <= p && p < 14)&&(modo[p] == OUTPUT)) {
			v = digital[p];
		} else {
			Serial.print("#> ERROR: getDigitalVal(");
			Serial.print(p);
			Serial.println(")");
		}
		return v;
	}
	void setDigitalVal(int p, int v) {
		if ((0 <= p && p < 14)&&(v == LOW || v == HIGH)&&(modo[p] == INPUT)) {
			digital[p] = v;
		} else {
			Serial.print("#> ERROR: setDigitalVal(");
			Serial.print(p);
			Serial.print(", ");
			Serial.print(v);
			Serial.println(")");
		}
	}
	void setAnalogVal(int p, int v) {
		if ((0 <= p && p < 6)&&(0 <= v && v <= 1023)) {
			analogico[p] = v;
		} else {
			Serial.print("#> ERROR: setAnalogVal(");
			Serial.print(p);
			Serial.print(", ");
			Serial.print(v);
			Serial.println(")");
		}
	}
};
static Arduino_ Arduino;
} // namespace internal__
//------------------------------------
namespace simulacion__ {
	using internal__::Serial;
	unsigned long millis() {
		return internal__::Arduino.millis();
	}
	int getDigitalVal(int p) {
		return internal__::Arduino.getDigitalVal(p);
	}
	void setDigitalVal(int p, int v) {
		internal__::Arduino.setDigitalVal(p, v);
	}
	void setAnalogVal(int p, int v) {
		internal__::Arduino.setAnalogVal(p, v);
	}
} // namespace simulacion__
//------------------------------------
namespace user__ {
	using internal__::Serial;
	using internal__::String;
	unsigned long millis() {
		return internal__::Arduino.millis();
	}
	void delay(unsigned long ms) {
		internal__::Arduino.delay(ms);
	}
	void pinMode(int p, int m) {
		internal__::Arduino.pinMode(p, m);
	}
	void digitalWrite(int p, int v) {
		internal__::Arduino.digitalWrite(p, v);
	}
	int digitalRead(int p) {
		return internal__::Arduino.digitalRead(p);
	}
	void analogWrite(int p, int v) {
		internal__::Arduino.analogWrite(p, v);
	}
	int analogRead(int p) {
		return internal__::Arduino.analogRead(p);
	}
	unsigned long micros() {
		Serial.println("#~ ERROR: unsupported operation: micros()");
		return 0;
	}
	void delayMicroseconds(unsigned long) {
		Serial.println("#~ ERROR: unsupported operation: delayMicroseconds()");
	}
	template <typename Tipo>
	inline Tipo min(Tipo v1, Tipo v2) { return v1 < v2 ? v1 : v2 ; }
	template <typename Tipo>
	inline Tipo max(Tipo v1, Tipo v2) { return v1 > v2 ? v1 : v2 ; }
} // namespace user__
//--------------------------------------------------------------------------
//-- Programa Principal ----------------------------------------------------
//--------------------------------------------------------------------------
int main(int argc, const char* argv[])
{
	unsigned long fin = 0;
	if ((argc == 2)&&(isdigit(argv[1][0]))) {
		fin = strtoul(argv[1], 0, 0);
	}
	if (fin < 10 || fin > 5000000) {
		fin = 500000;
	}
	internal__::Serial.println("#-------------------------------------------");
	simulacion__::simulacion_setup();
	user__::setup();
	while (internal__::Arduino.millis() < fin) {
		simulacion__::simulacion_loop();
		user__::loop();
		internal__::Arduino.inc_tiempo();
	}
	internal__::Serial.println("#-------------------------------------------");
}
#undef MAX_TEMP
//--------------------------------------------------------------------------
//-- Simulacion ------------------------------------------------------------
//--------------------------------------------------------------------------
namespace simulacion__ {
	// variacion de 2 grados centigrados en 5 segundos
	static const double INCR_TMP_MS = 4.0/1000.0;
	static const unsigned long PERIODO_SIMUL_MS = 100;
	static const unsigned RELAY_PIN = 2;
	static const unsigned TEMPERATURE_PIN = 0;
	struct Simulacion {
		unsigned long last_ms;
		double val;
	};
	static Simulacion simulacion;
	static void simulacion_setup() {
		simulacion.last_ms = -PERIODO_SIMUL_MS;
		simulacion.val = 512;
		setAnalogVal(TEMPERATURE_PIN, simulacion.val);
	}
	static void simulacion_loop() {
		unsigned long curr_ms = millis();
		if (curr_ms - simulacion.last_ms >= PERIODO_SIMUL_MS) {
			simulacion.last_ms += PERIODO_SIMUL_MS;
			if (curr_ms != 0) {
				int rele = getDigitalVal(RELAY_PIN);
				double inc_tmp = (PERIODO_SIMUL_MS
					* (rele == LOW ? -INCR_TMP_MS : +INCR_TMP_MS));
				simulacion.val += inc_tmp;
				simulacion.val = (simulacion.val < 0 ? 0 :
					(simulacion.val > 1023 ? 1023 : simulacion.val) );
				setAnalogVal(TEMPERATURE_PIN, simulacion.val);
			}
		}
	}
} // namespace simulacion__

//------------------------------------

namespace user__ {
	#include <math.h>
	const char* SALIDA = "bin/output/output16.plt";

	float val2tmp(int val, float T0, float R0, float B) {
		float t, r;

		r = ((1023.0 * 1000) / float(val)) - 1000;
		T0 += 273.15;
		t = 1.0 / ((1.0 / T0) + (log(r / R0) / B));

		return t - 273.15;
	}

	// ================================
	// Vars
	
	const float THERMISTOR_T0 = 25.0;
	const float THERMISTOR_R0 = 10000.0;
	const float THERMISTOR_B = 3977.0;
	const float KP = 0.4044;
	const float KI = 0.0100;
	const float KD = 3.3718;
	const unsigned long MONITOR_PERIOD = 1000;
	const unsigned long TEMPERATURE_PERIOD = 1000;
	const unsigned long RELAY_PERIOD = 5000;
	const byte RELAY_PIN = 2;
	const byte RED_PIN = 3;
	const byte GREEN_PIN = 5;
	const byte TEMPERATURE_PIN = 0;
	const float DESIRED_TEMP = 38.0;
	const float MIN_TEMP = 36.0;
	const float MAX_TEMP = 40.0;
	
	// ================================
	// Thermistor
	
	struct Temperature {
		unsigned long last_ms;
		float val;
	};

	void setup_temperature(unsigned long curr_ms, struct Temperature& tmp) {
		pinMode(GREEN_PIN, OUTPUT);
		tmp.last_ms = curr_ms - TEMPERATURE_PERIOD;
		tmp.val = 0;
	}

	void loop_temperature(unsigned long curr_ms, struct Temperature& tmp) {
		if (curr_ms - tmp.last_ms >= TEMPERATURE_PERIOD) {
			tmp.last_ms += TEMPERATURE_PERIOD;
			int val = analogRead(TEMPERATURE_PIN);
			tmp.val = val2tmp(val, THERMISTOR_T0, THERMISTOR_R0, THERMISTOR_B);
			byte est_tmp = (MIN_TEMP <= tmp.val && tmp.val <= MAX_TEMP) ? HIGH : LOW;
			digitalWrite(GREEN_PIN, est_tmp);
		}
	}

	// ================================
	// Relay

	struct Relay {
		unsigned long last_ms;
		unsigned long duty_cycle;
		float error;
		float last_error;
		float integral;
		float derivada;
		float pid;
		byte status;
	};

	void setup_relay(unsigned long curr_ms, struct Relay& relay) {
		pinMode(RELAY_PIN, OUTPUT);
		pinMode(RED_PIN, OUTPUT);
		relay.last_ms = curr_ms - RELAY_PERIOD;
		relay.status = LOW;
		relay.duty_cycle = 0;
		relay.error = 0;
		relay.last_error = relay.error;
		relay.integral = 0;
		relay.derivada = 0;
		relay.pid = 0;
	}

	void loop_relay(unsigned long curr_ms, float tmp_val, struct Relay& relay) {
		if (curr_ms - relay.last_ms >= RELAY_PERIOD) {
			relay.error = DESIRED_TEMP - tmp_val;
			relay.integral = relay.integral + (relay.error * RELAY_PERIOD * 0.001);
			relay.derivada = (relay.error - relay.last_error) / (RELAY_PERIOD * 0.001);
			relay.pid = KP * relay.error + KI * relay.integral + KD * relay.derivada;
			relay.last_error = relay.error;

			relay.last_ms += RELAY_PERIOD;
			
			if (relay.pid <= 0) {
				relay.status = LOW;
				relay.duty_cycle = 0 * RELAY_PERIOD / 100;
			} else if (relay.pid > 0 && relay.pid < 5) {
				relay.status = HIGH;
				relay.duty_cycle = (20 * relay.pid) * RELAY_PERIOD / 100;
			} else {
				relay.status = HIGH;
				relay.duty_cycle = RELAY_PERIOD;
			}

			digitalWrite(RELAY_PIN, relay.status);
			digitalWrite(RED_PIN, relay.status);
		}

		if (curr_ms - relay.last_ms >= relay.duty_cycle) {
			relay.duty_cycle = relay.last_ms + RELAY_PERIOD;
			relay.status = LOW;
			digitalWrite(RELAY_PIN, relay.status);
			digitalWrite(RED_PIN, relay.status);
		}
	}

	// ================================
	// Monitor

	struct Monitor {
		unsigned long last_ms;
	};

	void setup_monitor(unsigned long curr_ms, struct Monitor& mtr) {
		Serial.begin(9600);

		if (Serial) {
			Serial.print("#> SIMULACION Arduino Control de Temperatura PID ");
			Serial.print(KP);
			Serial.print(" ");
			Serial.print(KI);
			Serial.print(" ");
			Serial.print(KD);
			Serial.println(" (5000ms)");
			Serial.println("#$ -y20:45 -w4 -l36 -l38 -l40 -tTemperatura -tCalefactor");
		}

		mtr.last_ms = curr_ms - MONITOR_PERIOD;
	}

	void loop_monitor(unsigned long curr_ms, struct Monitor& mtr, const struct Temperature& tmp, const struct Relay& relay) {
		if (curr_ms - mtr.last_ms >= MONITOR_PERIOD) {
			mtr.last_ms += MONITOR_PERIOD;

			if (Serial) {
				Serial.print(tmp.val);
				Serial.print(" ");
				Serial.println(relay.status);
			}
		}
	}

	// ================================
	// Main loop

	struct Monitor mtr;
	struct Temperature tmp;
	struct Relay relay;

	void setup() {
		unsigned long curr_ms = millis();
		setup_temperature(curr_ms, tmp);
		setup_relay(curr_ms, relay);
		setup_monitor(curr_ms, mtr);
	}

	void loop() {
		unsigned long curr_ms = millis();
		loop_temperature(curr_ms, tmp);
		loop_relay(curr_ms, tmp.val, relay);
		loop_monitor(curr_ms, mtr, tmp, relay);
	}
}
