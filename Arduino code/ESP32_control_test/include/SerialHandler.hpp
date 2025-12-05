#ifndef SERIAL_HANDLER_HPP
#define SERIAL_HANDLER_HPP

#include "MotorHandler.hpp"
#include "SystemConfig.hpp"
#include <Arduino.h>
#include <ArduinoJson.h>

class SerialHandler {
private:
    MotorHandler& motor;
    String        inputBuffer;

public:
    SerialHandler( MotorHandler& m ) : motor( m ) {
        inputBuffer.reserve( 200 );
    }

    void setup() {
        Serial.begin( 115200 );
    }

    void update() {
        while ( Serial.available() ) {
            char c = Serial.read();
            if ( c == '\n' ) {
                processCommand( inputBuffer );
                inputBuffer = "";
            }
            else if ( c != '\r' ) {
                inputBuffer += c;
            }
        }
    }

    void processCommand( const String& json ) {
        JsonDocument         doc;
        DeserializationError error = deserializeJson( doc, json );

        if ( error ) {
            Serial.println( "{\"error\":\"Invalid JSON\"}" );
            return;
        }

        const char* cmd = doc[ "cmd" ];
        if ( !cmd )
            return;

        if ( strcmp( cmd, "MOVE" ) == 0 ) {
            if ( doc[ "val" ].is< float >() ) {
                float dist = doc[ "val" ];
                motor.setTargetDistance( dist );
            }
        }
        else if ( strcmp( cmd, "HOME" ) == 0 ) {
            motor.home();
        }
        else if ( strcmp( cmd, "STOP" ) == 0 ) {
            motor.setTargetPosition( motor.getPosition() );
            motor.stopMotor();
        }
    }

    void sendStatus() {
        JsonDocument doc;

        doc[ "pos" ]      = motor.getPosition();
        doc[ "target" ]   = motor.getTarget();
        doc[ "dist" ]     = motor.getDistanceMM();
        doc[ "pressure" ] = analogRead( PRESSURE_SENSOR_PIN );
        doc[ "btn_ctl" ]  = digitalRead( BUTTON_CONTROL_PIN );
        doc[ "btn_home" ] = digitalRead( BUTTON_HOME_PIN );

        serializeJson( doc, Serial );
        Serial.println();
    }
};

#endif  // SERIAL_HANDLER_HPP
