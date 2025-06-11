/*
 *   Created on: Apr 18, 2025
 *      Author: Phoenix Cardwell
 *      UCONN Formula SAE 2025
 */

#include <screenHandle.h>
#include <stdlib.h>
#include "screen.h"
#include <string>

void ScreenHandler::startScreen(){
	_screens[_curScreen]->titleScreen();
	clear_display();
	_screens[_curScreen]->Display();
}
void ScreenHandler::nextScreen(){
	_curScreen++; // go to next screen
	_curScreen %= (int)_screens.size(); // if past last screen go back to first screen
	_screens[_curScreen]->titleScreen();
	clear_display();

	_screens[_curScreen]->Display(); // update display ?
	// add edge case to loop back to beginning
}

void ScreenHandler::prevScreen(){
	_curScreen--; // go to last screen
	if(_curScreen < 0) _curScreen = (int) _screens.size() - 1; // if past first screen go to last screen
	_screens[_curScreen]->titleScreen();
	clear_display();
	_screens[_curScreen]->Display(); // update display ?
	// add edge case to loop back to beginning
}

void ScreenHandler::handle()
{
	_screens[_curScreen]->Display();
}

void ScreenHandler::addScreen(ScreenPanel* screen)
{
	_screens.push_back(screen);
}

// ouh should the boot screen a screen
// probably not because we don't need to flip through it

void WarmupScreen::Display()
{
	move_cursor(0,0);
	char str[32];
	sprintf(str, "ECT: %.3f", (((float)_ECT->getValue()) * 9/5 + 32));
	write_bytes(str);
	move_cursor(0,1);
	sprintf(str, "vBat: %.3f", (float)_vBat->getValue()/1000);
	write_bytes(str);
	move_cursor(0,2);
	sprintf(str, "Oil Pressure: %.3f", (float)_oilPressure->getValue()* 0.0145037738);
	write_bytes(str);
	move_cursor(0,3);
	sprintf(str, "Lambda: %.3f", (float)_lambda->getValue()/1000);
	write_bytes(str);

}

void WarmupScreen::titleScreen()
{
	clear_display();
	move_cursor(5,1);
	write_byte(129);
	write_bytes(" WARM-UP ");
	write_byte(129);

	move_cursor(0,2);
	for(int i = 0; i < 20; i++){
		write_byte(128);
	}

	HAL_Delay(1000);
}

void DrivingScreen::Display()
{
	move_cursor(0,0);
	char str[32];
	sprintf(str, "ECT: %.3f", (((float)_ECT->getValue()) * 1.8 + 32));
	write_bytes(str);
	move_cursor(0,1);
	sprintf(str, "Speed: %d MPH    ", _speed->getValue());
	write_bytes(str);

	move_cursor(0,2);
	sprintf(str, "Limp: %d", _limp->getValue());
	write_bytes(str);

	move_cursor(0,3);
	sprintf(str, "RPM: %d", _RPM->getValue());
	write_bytes(str);
}

void DrivingScreen::titleScreen()
{
	clear_display();
	move_cursor(5,1);
	write_byte(129);
	write_bytes(" DRIVING ");
	write_byte(129);

	move_cursor(0,2);
	for(int i = 0; i < 20; i++){
		write_byte(128);
	}

	HAL_Delay(1000);
}

std::string formatTime(int ms) {
    bool isNegative = ms < 0;
    ms = std::abs(ms);

    int hours = ms / 3600000;
    ms %= 3600000;
    int minutes = ms / 60000;
    ms %= 60000;
    int seconds = ms / 1000;
    ms %= 1000;

    char buffer[24]; // Extra space for minus sign and null terminator
    std::snprintf(
        buffer, sizeof(buffer), "%s%d:%02d:%02d.%03d",
        isNegative ? "-" : "",
        hours, minutes, seconds, ms
    );

    return std::string(buffer);
}

void LapTimeScreen::Display()
{
    char str[32];

    move_cursor(0, 0);
    std::string line0 = "Time: " + formatTime((unsigned int)_laptime->getTime());
    std::snprintf(str, sizeof(str), "%s", line0.c_str());
    //sprintf(str, "Time: %u", (unsigned int)_laptime->getTime());
    write_bytes(str);

    move_cursor(0, 1);
    std::string line1 = "Best: " + formatTime((unsigned int)_bestTime->getValue());
    std::snprintf(str, sizeof(str), "%s", line1.c_str());

   // sprintf(str, "Best: %s",formatTime( (unsigned int)_bestTime->getValue()));
    write_bytes(str);
//    std::string line2 = "Delta: " + formatTime((int)_diffBest->getValue());
//    std::snprintf(str, sizeof(str), "%s", line2.c_str());
    move_cursor(0,2);
    int delta = _bestTime->getValue() - _laptime ->getTime();
    std::string line2 = "Delta: " + formatTime((int)delta);
    std::snprintf(str, sizeof(str), "%s", line2.c_str());
    //sprintf(str, "Delta: %s", formatTime((int)delta));
    write_bytes(str);

    move_cursor(0, 3);
    std::snprintf(str, sizeof(str), "Lap#: %d", _laptime->getNum());
    write_bytes(str);

}
void LapTimeScreen::titleScreen()
{
	clear_display();
	move_cursor(5,1);
	write_byte(129);
	write_bytes(" LAP TIME ");
	write_byte(129);

	move_cursor(0,2);
	for(int i = 0; i < 20; i++){
		write_byte(128);
	}

	HAL_Delay(1000);
}
