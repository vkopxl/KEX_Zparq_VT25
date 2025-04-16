
// GLÖM INTE KOLLA OM DET ÄR PULL UP ELLER PULL DOWN KOPPLING SÅ ATT HIGH OCH LOW BLIR RÄTT I KODEN


// Tillståndsmaskinen startar i ”STOP”
int tillstand = 1;	

// Blinkande GUL lampa
unsigned long previousMillis = 0;
const long blinkInterval = 500;
bool blinkState = false;

bool avbryt_counter = false; 

// Långtryck på autotrim-knapp
unsigned long knappTryckStart = 0;
bool knappVarNere = false;
const unsigned long langTryckTid = 2000;

// Funktion för blinkande gul lampa
void blinkGulLampa() {
	unsigned long currentMillis = millis();
	if (currentMillis - previousMillis >= blinkInterval) {
		previousMillis = currentMillis;
    	blinkState = !blinkState;
       	digitalWrite(13, LOW); 			// RÖD av
    	digitalWrite(12, blinkState); 	// GUL växlar
    	digitalWrite(8, LOW);  			// GRÖN av
	}
}

void setup() {
    Serial.begin(9600);

    // Inputs
    pinMode(2, INPUT_PULLUP);  // Autotrim-knapp
    pinMode(4, INPUT_PULLUP);  // Knapp Ned
    pinMode(7, INPUT_PULLUP);  // Knapp Upp


    // Outputs
    pinMode(13, OUTPUT);  // LED RÖD
    pinMode(12, OUTPUT);  // LED GUL
    pinMode(8, OUTPUT);   // LED GRÖN
}


void loop() {

    bool nuvarandeKnapp = !digitalRead(2);  // Autotrim-knapp    
    bool nuvarandeKnapp_ned = !digitalRead(4);  // Manuell trim ned
    bool nuvarandeKnapp_upp = !digitalRead(7);  // Manuell trim upp 
    unsigned long nu = millis();

    // EDGE DETECTION FÖR AUTOTRIM-KNAPP
    static bool forraKnapp = true;
    bool auto_knappNer = (!forraKnapp && nuvarandeKnapp);  // Just tryckt
    bool auto_knappUpp = (forraKnapp && !nuvarandeKnapp);  // Just släppt
    forraKnapp = nuvarandeKnapp;

    // EDGE DETECTION FÖR MANUELL TRIM NED
    static bool forraKnapp_ned = true;
    bool manuell_trimned_knappNedtryckt = (!forraKnapp_ned && nuvarandeKnapp_ned);  // Just tryckt
    bool manuell_trimned_knappSlappt = (forraKnapp_ned && !nuvarandeKnapp_ned);  // Just släppt
    forraKnapp_ned = nuvarandeKnapp_ned;

    // EDGE DETECTION FÖR MANUELL TRIM UPP
    static bool forraKnapp_upp = true;
    bool manuell_trimupp_knappNedtryckt = (!forraKnapp_upp && nuvarandeKnapp_upp);  // Just tryckt
    bool manuell_trimupp_knappSlappt = (forraKnapp_upp && !nuvarandeKnapp_upp);  // Just släppt
    forraKnapp_upp = nuvarandeKnapp_upp;

    switch (tillstand) {

        case 1: { // STOP: RÖD tänd
            digitalWrite(13, HIGH); 	// RÖD
            digitalWrite(12, LOW);  	// GUL
            digitalWrite(8, LOW);   	// GRÖN

                if (auto_knappNer) {
                    Serial.println("STARTA");
                    tillstand = 2;
                }

            break;

        }

        case 2: { // Trimning pågår: GUL blinkar
            
            avbryt_counter = true; 

            blinkGulLampa();

                if (auto_knappNer) {
                    Serial.println("AVBRYT");
                }

                // Vänta på svar från Python
                if (Serial.available()) {
                    String message = Serial.readStringUntil('\n');
                    if (message == "KLAR") {
                        tillstand = 3;
                    }
                    else if (message == "AVBRUTEN") {
                        tillstand = 1;
                    }
                }

            break;   
        }

        case 3: { // OPTIMERAD: GRÖN tänd

            digitalWrite(13, LOW);  // RÖD
            digitalWrite(12, LOW);  // GUL
            digitalWrite(8, HIGH);  // GRÖN

                if (nuvarandeKnapp == LOW && !knappVarNere) {
                    knappVarNere = true;
                    knappTryckStart = millis();
                }

                if (nuvarandeKnapp == HIGH && knappVarNere) {
                    knappVarNere = false;
                    unsigned long knappTid = millis() - knappTryckStart;

                    if (knappTid >= langTryckTid) {
                        tillstand = 1; // Långt tryck
                    } 
                    else {
                        Serial.println("STARTA");
                        tillstand = 2; // Kort tryck
                    }
                }
            
            break;

        }
    }

    // MANUELL TRIM OVERRIDE

    // Om någon manuell knapp trycks ner, tvinga tillstånd 1
    if (manuell_trimned_knappSlappt && avbryt_counter) {

        Serial.println("AVBRYT");

        avbryt_counter = false; 

        tillstand = 1;
    }

    if (manuell_trimupp_knappSlappt && avbryt_counter) {

        Serial.println("AVBRYT");

        avbryt_counter = false; 

        tillstand = 1;
    }

    // Tryckt ner trim UPP
    if (manuell_trimupp_knappSlappt) {
        Serial.println("MANUELL TRIM UPP");
        tillstand = 1;
    }

    // Släppt trim UPP
    if (manuell_trimupp_knappNedtryckt) {
        Serial.println("AVBRYT MANUELL TRIM UPP");
        tillstand = 1;
    }

    // Tryckt ner trim NED
    if (manuell_trimned_knappSlappt) {
        Serial.println("MANUELL TRIM NED");
        tillstand = 1;
    }

    // Släppt trim NED
    if (manuell_trimned_knappNedtryckt) {
        Serial.println("AVBRYT MANUELL TRIM NED");
        tillstand = 1;
    }

}