#include "inc/Globals.hpp"
#include "../Configuration.hpp"
#include "Utility.hpp"
#include "Mount.hpp"
#include "MeadeCommandProcessor.hpp"
#include "WifiControl.hpp"

#if USE_GPS == 1
bool gpsAqcuisitionComplete(int &indicator);  // defined in c72_menuHA_GPS.hpp
#endif
/////////////////////////////////////////////////////////////////////////////////////////
//
// Serial support
//
// The Serial protocol implemented here is the Meade LX200 Classic protocol with some extensions.
// The Meade protocol commands start with a colon and end with a hash.
// The first letter determines the family of functions (G for Get, S for Set, M for Movement, etc.)
//
// The set of Meade features implemented are:
//
//------------------------------------------------------------------
// INITIALIZE FAMILY
//
// :I#
//      Description:
//        Initialize Scope
//      Information:
//        This puts the Arduino in Serial Control Mode and displays RA on line 1 and
//        DEC on line 2 of the display. Serial Control Mode can be ended manually by
//        pressing the SELECT key, or programmatically with the :Qq# command.
//      Returns:
//        nothing
//
//------------------------------------------------------------------
// SYNC CONTROL FAMILY
//
// :CM#
//      Description:
//        Synchronize Declination and Right Ascension.
//      Information:
//        This tells the scope what it is currently pointing at. The scope synchronizes
//        to the current target coordinates
//      Remarks:
//        Set with ":Sd#" and ":Sr#"
//      Returns:
//        "NONE#"
//
//------------------------------------------------------------------
// DISTANCE FAMILY
//
// :D#
//      Description:
//        Query Mount Status
//      Information:
//        This queries the mount for its slewing status
//      Returns:
//        "|#" if slewing
//        " #" if not
//
//------------------------------------------------------------------
// GPS FAMILY
//
// :gT#
//      Description:
//        Set Mount Time
//      Information:
//        Attempts to set the mount time and location from the GPS for 2 minutes. This is essentially a
//        blocking call, no other activities take place (except tracking, but only if interrupt-driven).
//      Remarks:
//        Use ":Gt#" and ":Gg#" to retrieve Lat and Long,
//      Returns:
//        "1" if the data was set
//        "0" if not (timedout)
//
// :gTnnn#
//      Description:
//        Set Mount Time w/ timeout
//      Information:
//        Attempts to set the mount time and location from the GPS with a custom timeout. This is also blocking
//        but by using a low timeout, you can avoid long pauses and let the user know that it's not ready yet.
//      Returns:
//        "1" if the data was set
//        "0" if not (timedout)
//      Parameters:
//        "nnn" is an integer defining the number of milliseconds to wait for the GPS to get a bearing
//
//------------------------------------------------------------------
// GET FAMILY
//
// :GVP#
//      Description:
//        Get the Product Name
//      Returns:
//        "OpenAstroTracker#"
//
// :GVN#
//      Description:
//        Get the Firmware Version Number
//      Returns:
//        "V1.major.minor#" from version.h
//
// :Gd#
//      Description:
//        Get Target Declination
//      Returns:
//        "sDD*MM'SS#"
//      Parameters:
//        "s" is + or -
//        "DD" is degrees
//        "MM" is minutes
//        "SS" is seconds
//
// :GD#
//      Description:
//        Get Current Declination
//      Returns:
//        "sDD*MM'SS#"
//      Parameters:
//        "s" is + or -
//        "DD" is degrees
//        "MM" is minutes
//        "SS" is seconds.
//
// :Gr#
//      Description:
//        Get Target Right Ascension
//      Returns:
//        HH:MM:SS#
//      Parameters:
//        "HH" is hour
//        "MM" is minutes
//        "SS" is seconds
//
// :GR#
//      Description:
//        Get Current Right Ascension
//      Returns:
//        "HH:MM:SS#"
//      Parameters:
//        "HH" is hour
//        "MM" is minutes
//        "SS" is seconds
//
// :Gt#
//      Description:
//        Get Site Latitude
//      Returns:
//        "sDD*MM#"
//      Parameters:
//        "s" is + or -
//        "DD" is the latitude in degrees
//        "MM" the minutes
//
// :Gg#
//      Description:
//        Get Site Longitude
//      Returns:
//        "sDDD*MM#"
//      Parameters:
//        "s" is the sign of the longitude
//        "DDD" is the degrees
//        "MM" is the minutes
//      Remarks:
//        Note that this is the actual longitude, but east coordinates are negative (opposite of normal cartographic coordinates)
//
// :Gc#
//      Description:
//        Get current Clock format
//      Returns:
//        "24#"
//
// :GG#
//      Description:
//        Get offset to UTC time
//      Returns:
//        "sHH#"
//      Parameters:
//        "s" is the sign
//        "HH" is the number of hours
//      Remarks
//        Note that this is NOT simply the timezone offset you are in (like -8 for Pacific Standard Time), it is the negative of it. So how many hours need to be added to your local time to get to UTC.
//
// :Ga#
//      Description:
//        Get local time in 12h format
//      Returns:
//        "HH:MM:SS#"
//      Parameters:
//        "HH" are hours (modulo 12)
//        "MM" are minutes
//        "SS" are seconds of the local time
//
// :GL#
//      Description:
//        Get local time in 24h format
//      Returns:
//        "HH:MM:SS#"
//      Parameters:
//        "HH" are hours
//        "MM" are minutes
//        "SS" are seconds of the local time
//
// :GC#
//      Description:
//        Get current date
//      Returns:
//        "MM/DD/YY#"
//      Parameters:
//        "MM" is the month (1-12)
//        "day" is the day (1-31)
//        "year" is the lower two digits of the year
//
// :GM#
//      Description:
//        Get Site Name 1
//      Returns:
//        "OAT1#"
//
// :GN#
//      Description:
//        Get Site Name 2
//      Returns:
//        "OAT2#"
//
// :GO#
//      Description:
//        Get Site Name 3
//      Returns:
//        OAT2#
//
// :GP#
//      Description:
//        Get Site Name 4
//      Returns:
//        OAT4#
//
// :GT#
//      Description:
//        Get tracking rate
//      Returns:
//        60.0#
//
//------------------------------------------------------------------
// GET EXTENSIONS
//
// :GIS#
//      Description:
//        Get DEC or RA Slewing
//      Returns:
//        "1#" if either RA or DEC is slewing
//        "0#" if not
//
// :GIT#
//      Description:
//        Get Tracking
//      Returns:
//        "1#" if tracking is on
//        "0#" if not
//
// :GIG#
//      Description:
//        Get Guiding
//      Returns:
//        "1#" if currently guiding
//        "0#" if not
//
// :GX#
//      Description:
//        Get Mount Status
//      Information:
//         String reflecting the mounts' status. The string is a comma-delimited list of statuses
//      Returns:
//        "Idle,--T--,11219,0,927,071906,+900000,#"
//      Parameters:
//        [0] The mount status. One of 'Idle', 'Parked', 'Parking', 'Guiding', 'SlewToTarget', 'FreeSlew', 'ManualSlew', 'Tracking', 'Homing'
//        [1] The motion state.
//        [2] The RA stepper position
//        [3] The DEC stepper position
//        [4] The Tracking stepper position
//        [5] The current RA position
//        [6] The current DEC position
//      Remarks:
//        The motion state
//        First character is RA slewing state ('R' is East, 'r' is West, '-' is stopped).
//        Second character is DEC slewing state ('d' is North, 'D' is South, '-' is stopped).
//        Third character is TRK slewing state ('T' is Tracking, '-' is stopped).
//        Fourth character is AZ slewing state ('Z' and 'z' is adjusting, '-' is stopped).
//        Fifth character is ALT slewing state ('A' and 'a' is adjusting, '-' is stopped).
//        Az and Alt are optional. The string may only be 3 characters long
//
//------------------------------------------------------------------
// SET FAMILY
//
// :SdsDD*MM:SS#
//      Description:
//        Set Target Declination
//      Information:
//        This sets the target DEC. Use a Movement command to slew there.
//      Parameters:
//        "s" is + or -
//        "DD" is degrees
//        "MM" is minutes
//        "SS" is seconds
//      Returns:
//        "1" if successfully set
//        "0" otherwise
//
// :SrHH:MM:SS#
//      Description:
//        Set Right Ascension
//      Information:
//        This sets the target RA. Use a Movement command to slew there.
//      Returns:
//        "1" if successfully set
//        "0" otherwise
//      Parameters:
//        "HH" is hours
//        "MM" is minutes
//        "SS" is seconds
//
// :StsDD*MM#
//      Description:
//        Set Site Latitude
//      Information:
//        This sets the latitude of the location of the mount.
//      Returns:
//        "1" if successfully set
//        "0" otherwise
//      Parameters:
//        "s" is the sign ('+' or '-')
//        "DD" is the degree (90 or less)
//        "MM" is minutes
//
// :SgsDDD*MM#
//      Description:
//        Set Site Longitude
//      Information:
//        This sets the longitude of the location of the mount.
//      Returns:
//        "1" if successfully set
//        "0" otherwise
//      Parameters:
//        "s" (optional) is the sign of the longitude (see Remarks)
//        "DDD" is the number of degrees
//        "MM" is the minutes
//      Remarks:
//        When a sign is provided, longitudes are interpreted as given, with zero at Greenwich but negative coordinates going east (opposite of normal cartographic coordinates)
//        When a sign is not provided, longitudes are from 0 to 360 going WEST with 180 at Greenwich. So 369 is 179W and 1 is 179E. 190 would be 10W and 170 would be 10E.
//
// :SGsHH#
//      Description:
//        Set Site UTC Offset
//      Information:
//        This sets the offset of the timezone in which the mount is in hours from UTC.
//      Returns:
//        "1"
//      Parameters:
//        "s" is the sign
//        "HH" is the number of hours
//
// :SLHH:MM:SS#
//      Description:
//        Set Site Local Time
//      Information:
//        This sets the local time of the timezone in which the mount is located.
//      Returns:
//        "1"
//      Parameters:
//        "HH" is hours
//        "MM" is minutes
//        "SS" is seconds
//
// :SCMM/DD/YY#
//      Description:
//        Set Site Date
//      Information:
//        This sets the date
//      Returns:
//        "1Updating Planetary Data#                              #"
//      Parameters:
//        "MM" is the month
//        "DD" is the day
//        "YY" is the year since 2000
//
//------------------------------------------------------------------
// SET Extensions
//
// :SHHH:MM#
//      Description:
//        Set HA (Hour Angle of Polaris)
//      Information:
//        This sets the scopes HA, which should be that of Polaris.
//      Returns:
//        "1" if successfully set
//        "0" otherwise
//      Parameters:
//        "HH" is hours
//        "MM" is minutes
//
// :SHP#
//      Description:
//        Set Home Point
//      Information:
//        This sets the current orientation of the scope as its home point.
//      Returns:
//        "1"
//
// :SHLHH:MM#
//      Description:
//        Set LST Time
//      Information:
//        This sets the scopes LST (and HA).
//      Returns:
//        "1" if successfully set
//        "0" otherwise
//      Parameters:
//        "HH" is hours
//        "MM" is minutes
//
// :SYsDD*MM:SS.HH:MM:SS#
//      Description:
//        Synchronize Declination and Right Ascension.
//      Information:
//        This tells the scope what it is currently pointing at.
//      Returns:
//        "1" if successfully set
//        "0" otherwise
//      Parameters:
//        "s" is + or -
//        "DD" is degrees
//        "HH" is hours
//        "MM" is minutes
//        "SS" is seconds
//
//------------------------------------------------------------------
// RATE CONTROL FAMILY
//
// :Rs#
//      Description:
//        Set Slew rate
//      Parameters:
//        "s" is one of 'S', 'M', 'C', or 'G' in order of decreasing speed
//      Returns:
//        nothing
//
//------------------------------------------------------------------
// MOVEMENT FAMILY
//
// :MS#
//      Description:
//        Start Slew to Target (Asynchronously)
//      Information:
//        This starts slewing the scope to the target RA and DEC coordinates and returns immediately.
//      Returns:
//        "0"
//
//------------------------------------------------------------------
// MOVEMENT EXTENSIONS
//
// :MGdnnnn#
//      Description:
//        Run a Guide pulse
//      Information:
//        This runs the motors at increased speed for a short period of time.
//      Parameters:
//        "d" is one of 'N', 'E', 'W', or 'S'
//        "nnnn" is the duration in ms
//      Returns:
//        "1"
//
// :MTs#
//      Description:
//        Set Tracking mode
//      Information:
//        This turns the scopes tracking mode on or off.
//      Parameters:
//        "s" is "1" to turn on Tracking and "0" to turn it off
//      Returns:
//        "1"
//
// :Mc#
//      Description:
//        Start slewing
//      Information:
//        This starts slewing the mount in the given direction.
//      Parameters:
//        "c" is one of 'n', 'e', 'w', or 's'
//      Returns:
//        nothing
//
// :MXxnnnnn#
//      Description:
//        Move stepper
//      Information:
//        This starts moving one of the steppers by the given amount of steps and returns immediately.
//      Parameters:
//        "x" is the stepper to move (r for RA, d for DEC, f for FOC, z for AZ, t for ALT)
//        "nnnn" is the number of steps
//      Returns:
//        "1" if successfully scheduled
//
// :MHRxn#
//      Description:
//        Home RA stepper via Hall sensor
//      Information:
//        This attempts to find the hall sensor and to home the RA ring accordingly.
//      Parameters:
//        "x" is either 'R' or 'L' and determines the direction in which the search starts (L is CW, R is CCW).
//        "n" (Optional) is the maximum number of hours to move while searching for the sensor location. Defaults to 2h. Limited to the range 1h-5h.
//      Remarks:
//        The ring is first moved 30 degrees (or the given amount) in the initial direction. If no hall sensor is encountered,
//        it will move twice the amount (60 degrees by default) in the opposite direction.
//        If a hall sensor is not encountered during that slew, the homing exits with a failure.
//        If the sensor is found, it will slew to the middle position of the Hall sensor trigger range and then to the offset
//        specified in the Home offset position (set with the ":XSHRnnnn#" command).
//        If the RA ring is positioned such that the Hall sensor is already triggered when the command is received, the mount will move
//        the RA ring off the trigger in the opposite direction specified for a max of 15 degrees before searching 30 degrees in the
//        specified direction.
//      Returns:
//        "1" returns if search is started
//        "0" if homing has not been enabled in the local config
//
// :MAZn.nn#
//      Description:
//        Move Azimuth
//      Information:
//        If the scope supports automated azimuth operation, move azimuth by n.nn arcminutes
//      Parameters:
//        "n.nn" is a signed floating point number representing the number of arcminutes to move the mount left or right
//      Returns:
//        nothing
//
// :MALn.nn#
//      Description:
//        Move Altitude
//      Information:
//        If the scope supports automated altitude operation, move altitude by n.nn arcminutes
//      Parameters:
//        "n.nn" is a signed floating point number representing the number of arcminutes to raise or lower the mount.
//      Returns:
//        nothing
//
//------------------------------------------------------------------
// HOME FAMILY
//
// :hP#
//      Description:
//        Park Scope and stop motors
//      Information:
//        This slews the scope back to it's home position (RA ring centered, DEC
//        at 90, basically pointing at celestial pole) and stops all movement (including tracking).
//      Returns:
//        nothing
//
// :hF#
//      Description:
//        Move Scope to Home position
//      Information:
//        This slews the scope back to its home position (RA ring centered, DEC
//        at 90, basically pointing at celestial pole). Mount will keep tracking.
//      Returns:
//        nothing
//
//------------------------------------------------------------------
// PARK Extensions
//
// :hU#
//      Description:
//        Unpark Scope
//      Information:
//        This currently simply turns on tracking.
//      Returns:
//        "1"
//
//------------------------------------------------------------------
// QUIT MOVEMENT FAMILY
//
// :Q#
//      Description:
//        Stop all motors
//      Information:
//        This stops all motors, including tracking. Note that deceleration curves are still followed.
//      Returns:
//        "1" when all motors have stopped
//
// :Qd#
//      Description:
//        Stop Slewing
//      Information:
//        Stops slew in specified direction where d is n, s, e, w, a (the first four are the cardinal directions, a stands for All).
//      Returns:
//        nothing
//
//------------------------------------------------------------------
// QUIT MOVEMENT Extensions
//
// :Qq#
//      Description:
//        Disconnect, Quit Control mode
//      Information:
//        This quits Serial Control mode and starts tracking.
//      Returns:
//        nothing
//
//------------------------------------------------------------------
// EXTRA OAT FAMILY - These are meant for the PC control app
//
// :XFR#
//      Description:
//        Perform a Factory Reset
//      Information:
//        Clears all the EEPROM settings
//      Returns:
//        "1#"
//
// :XDnnn#
//      Description:
//        Run drift alignment
//      Information:
//        This runs a drift alignment procedure where the mounts slews east, pauses, slews west and pauses.
//        Where nnn is the number of seconds the entire alignment should take. The call is blocking and will
//        only return once the drift alignment is complete.
//      Returns:
//        nothing
//
// :XL0#
//      Description:
//        Turn off the Digital level
//      Returns:
//        "1#" if successful
//        "0#" if there is no Digital Level
//
// :XL1#
//      Description:
//        Turn on the Digital level
//      Returns:
//        "1#" if successful
//        "0#" if there is no Digital Level
//
// :XLGR#
//      Description:
//        Digital Level - Get Reference
//      Information:
//        Gets the reference pitch and roll values of the mount (Digital Level addon). These
//        values are the values of the pitch and roll when the mount is level.
//      Returns:
//        "<pitch>,<roll>#"
//        "0#" if there is no Digital Level
//
// :XLGC#
//      Description:
//        Digital Level - Get Values
//      Information:
//        Gets the current pitch and roll values of the mount (Digital Level addon).
//      Returns:
//        "<pitch>,<roll>#"
//        "0#" if there is no Digital Level
//
// :XLGT#
//      Description:
//        Digital Level - Get Temperature
//      Information:
//        Get the current temperature in Celsius of the mount (Digital Level addon).
//      Returns:
//        "<temp>#"
//        "0#" if there is no Digital Level
//
// :XLSR#
//      Description:
//        Digital Level - Set Reference Roll
//      Information:
//        Sets the reference roll value of the mount (Digital Level addon). This is the value
//        at which the mount is level.
//      Returns:
//        "1#" if successful
//        "0#" if there is no Digital Level
//
// :XLSP#
//      Description:
//        Digital Level - Set Reference Pitch
//      Information:
//        Sets the reference pitch value of the mount (Digital Level addon). This is the value
//        at which the mount is level.
//      Returns:
//        "1#" if succsessful
//        "0#" if there is no Digital Level
//
// :XGB#
//      Description:
//        Get Backlash correction steps
//      Information:
//        Get the number of steps the RA stepper motor needs to overshoot and backtrack when slewing east.
//      Returns:
//        "integer#"
//
// :XGCn.nn*m.mm#
//      Description:
//        Get stepper motor positions for target
//      Information:
//        Get the positions of stepper motors when pointed at the given coordinates.
//      Parameters:
//        "n.nn" is the RA coordinate (0.0 - 23.999)
//        "m.mm" is the DEC coordinate (-90.00 - +90.00)
//        "ralong" is the stepper position of the RA stepper
//        "declong" is the stepper position of the DEC stepper
//      Returns:
//        "ralong,declong#"
//
// :XGR#
//      Description:
//        Get RA steps
//      Information:
//        Get the number of steps the RA stepper motor needs to take to rotate RA by one degree
//      Returns:
//        "float#"
//
// :XGD#
//      Description:
//        Get DEC steps
//      Information:
//        Get the number of steps the DEC stepper motor needs to take to rotate DEC by one degree
//      Returns:
//        "float#"
//
// :XGDL#
//      Description:
//        Get DEC limits
//      Information:
//        Get the lower and upper limits for the DEC stepper motor in steps
//      Returns:
//        "integer|integer#"
//
// :XGDP#
//      Description:
//        Get DEC parking position
//      Information:
//        Gets the number of steps from the home position to the parking position for DEC
//      Returns:
//        "long#"
//
// :XGS#
//      Description:
//        Get Tracking speed adjustment
//      Information:
//        Get the adjustment factor used to speed up (>1.0) or slow down (<1.0) the tracking speed of the mount.
//      Returns:
//        "float#"
//
// :XGT#
//      Description:
//        Get Tracking speed
//      Information:
//        Get the absolute tracking speed of the mount.
//      Returns:
//        "float#"
//
// :XGH#
//      Description:
//        Get HA (Hour Angle of Polaris)
//      Information:
//        Get the current HA of Polaris that the mount thinks it is.
//      Returns:
//        "HHMMSS#"
//
// :XGHR#
//      Description:
//        Get RA Homing offset
//      Information:
//        Get the RA ring homing offset for Hall sensor auto homing
//      Returns:
//        "n#" - the number of steps from the center of the hall sensor trigger range to the home position.
//
// :XGM#
//      Description:
//        Get Mount configuration settings
//      Returns:
//        "<board>,<RA Stepper Info>,<DEC Stepper Info>,<GPS info>,<AzAlt info>,<Gyro info>#"
//      Parameters:
//        "<board>" is one of the supported boards (currently Mega, ESP32, MKS)
//        "<Stepper Info>" is a pipe-delimited string of Motor type (NEMA or 28BYJ), Pulley Teeth, Steps per revolution)
//        "<GPS info>" is either NO_GPS or GPS, depending on whether a GPS module is present
//        "<AzAlt info>" is either NO_AZ_ALT, AUTO_AZ_ALT, AUTO_AZ, or AUTO_ALT, depending on which AutoPA stepper motors are present
//        "<Gyro info>" is either NO_GYRO or GYRO depending on whether the Digial level is present
//      Example:
//        "ESP32,28BYJ|16|4096.00,28BYJ|16|4096.00,NO_GPS,NO_AZ_ALT,NO_GYRO#"
//
// :XGMS#
//      Description:
//        Get Mount driver configuration
//      Returns:
//        "<RA driver>,<RA slewMS>,<RA trackMS>|<DEC driver>,<DEC slewMS>,<DEC guideMS>|#"
//      Parameters:
//        "<driver>" is one of the supported drivers: TU=TMC2209UART, TS=TMC2209STANDALONE, A=A4983
//        "<slewMS>" is the microstepping divider (1, 2, 4, 8, 15, 21, 64, 128, 256) used when slewing
//        "<trackMS>" is the microstepping divider (1, 2, 4, 8, 15, 21, 64, 128, 256) used when tracking RA
//        "<guideMS>" is the microstepping divider (1, 2, 4, 8, 15, 21, 64, 128, 256) used when guiding DEC
//      Example:
//        "TU,8,64|TU,16,64|#"
//
// :XGN#
//      Description:
//        Get network settings
//      Information:
//        Gets the current status of the Wifi connection. Reply only available when running on ESP boards.
//      Returns:
//        "1,<mode>,<status>,<hostname>,<ip>:<port>,<SSID>,<OATHostname>#" if Wifi is enabled
//        "0,#" if Wifi is not enabled
//
// :XGL#
//      Description:
//        Get LST
//      Information:
//        Get the current LST of the mount.
//      Returns:
//        "HHMMSS"
//
// :XSBn#
//      Description:
//        Set Backlash correction steps
//      Information:
//        Sets the number of steps the RA stepper motor needs to overshoot and backtrack when slewing east.
//      Returns:
//        nothing
//
// :XSHRnnn#
//      Description:
//        Set homing offset for RA ring from Hall sensor center
//      Information:
//        This offset is added to the position of the RA ring when it is centered on the hall sensor triggered range after running.
//        the RA homing command (:MHRx#)
//      Parameters:
//        "n" is the number of steps that are needed from the center of the Hall senser trigger range to the actual home position.
//      Returns:
//        nothing
//
// :XSRn.n#
//      Description:
//        Set RA steps
//      Information:
//        Set the number of steps the RA stepper motor needs to take to rotate by one degree.
//      Parameters:
//        "n.n" is the number of steps (only one decimal point is supported)
//      Returns:
//        nothing
//
// :XSDn.n#
//      Description:
//        Set DEC steps
//      Information:
//        Set the number of steps the DEC stepper motor needs to take to rotate by one degree.
//      Parameters:
//        "n.n" is the number of steps (only one decimal point is supported)
//      Returns:
//        nothing
//
// :XSDLUnnnnn#
//      Description:
//        Set DEC upper limit
//      Information:
//        Set the upper limit for the DEC stepper motor to the current position if no parameter is given,
//        otherwise to the given parameter.
//      Parameters:
//        "nnnnn" is the number of steps from home that the DEC ring can travel upwards
//      Returns:
//        nothing
//
// :XSDLu#
//      Description:
//        Clear DEC upper limit
//      Information:
//        Clears the upper limit for the DEC stepper motor
//      Returns:
//        nothing
//
// :XSDLLnnnnn#
//      Description:
//        Set DEC lower limit
//      Information:
//        Set the lowerlimit for the DEC stepper motor to the current position if no parameter is given,
//        otherwise to the given parameter.
//      Parameters:
//        "nnnnn" is the number of steps from home that the DEC ring can travel downwards
//      Returns:
//        nothing
//
// :XSDLl#
//      Description:
//        Clear DEC lower limit
//      Information:
//        Clear the lower limit for the DEC stepper motor
//      Returns: nothing
//
// :XSDPnnnn#
//      Description:
//        Set DEC parking position offset
//      Information:
//        This stores the number of steps needed to move from home to the parking position.
//      Returns: nothing
//
// :XSSn.nnn#
//      Description:
//        Set Tracking speed adjustment
//      Information:
//        Set the adjustment factor used to speed up "(>1.0)" or slow down "(<1.0)" the tracking speed of the mount
//      Parameters:
//        "n.nnn" is the factor to multiply the theoretical speed by
//      Returns:
//        nothing
//
// :XSTnnnn#
//      Description:
//        Set Tracking motor position (no movement)
//      Information:
//        This is purely a debugging aid. It is not recommended to call this unless you know what you are doing. It simply sets the internal tracking steps to the given value.
//      Parameters:
//        "nnn" is the stepper steps to set
//      Returns:
//        nothing
//
// :XSMn#
//      Description:
//        Set Manual Slewing Mode
//      Information:
//        Toggle the manual slewing mode state where the RA and DEC motors run at a constant speed
//      Parameters:
//        "n" is '1' to turn it on, otherwise turn it off
//      Returns:
//        nothing
//
// :XSXn.nnn#
//      Description:
//        Set RA Speed
//      Information:
//        Set RA manual slewing speed in degrees/sec immediately. Max is around 2.5 degs/s
//      Returns:
//        nothing
//      Remarks:
//        Must be in manual slewing mode.
//
// :XSYn.nnn#
//      Description:
//        Set DEC Speed
//      Information:
//        Set DEC manual slewing speed in degrees/sec immediately. Max is around 2.5 degs/s
//      Returns:
//        nothing
//      Remarks:
//        Must be in manual slewing mode.
//
//------------------------------------------------------------------
// FOCUS FAMILY
//
// :F+#
//      Description:
//        Start Focuser moving inward (toward objective)
//      Information:
//        Continues pull in until stopped
//      Returns:
//        nothing
//
// :F-#
//      Description:
//        Pull out
//      Information:
//        Continues pull out until stopped
//      Returns:
//        nothing
//
// :Fn#
//      Description:
//        Set speed factor
//      Information:
//        Set focuser speed to <n> where <n> is an ASCII digit 1..4. 1 is slowest, 4 i fastest
//      Returns:
//        nothing
//
// :FS#
//      Description:
//        Set slowest speed factor
//      Information:
//        Set focuser to the slowest speed it can use
//      Returns:
//        nothing
//
// :FF#
//      Description:
//        Set fastest speed factor
//      Information:
//        Set focuser speed to the fastest speed it can use
//      Returns:
//        nothing
//
// :Fp#
//      Description:
//        Get position
//      Information:
//        Get the current position of the focus stepper motor
//      Returns:
//        "nnn#" "nnn" is the current position of the stepper
//
//
// :FPnnn#
//      Description:
//        Set position
//      Information:
//        Sets the current position of the focus stepper motor
//      Returns:
//        "1"
//      Parameters:
//        "nnn" is the new position of the stepper. The stepper is not moved.
//
// :FB#
//      Description:
//        Get focuser state
//      Information:
//        Gets the state of the focuser stepper.
//      Returns:
//        "0" if the focuser is idle
//        "1" if the focuser is moving
//
// :FQ#
//      Description:
//        Stop focuser
//      Information:
//        Stops the stepper motor of the focuser.
//      Returns:
//        nothing
//
//------------------------------------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////

MeadeCommandProcessor *MeadeCommandProcessor::_instance = nullptr;

/////////////////////////////
// Create the processor
/////////////////////////////
MeadeCommandProcessor *MeadeCommandProcessor::createProcessor(Mount *mount)
{
    _instance = new MeadeCommandProcessor(mount);
    return _instance;
}

/////////////////////////////
// Get the singleton
/////////////////////////////
MeadeCommandProcessor *MeadeCommandProcessor::instance()
{
    return _instance;
}

/////////////////////////////
// Constructor
/////////////////////////////
MeadeCommandProcessor::MeadeCommandProcessor(Mount *mount)
{
    _mount = mount;
}

/////////////////////////////
// INIT
/////////////////////////////
String MeadeCommandProcessor::handleMeadeInit(String inCmd)
{
    inSerialControl = true;
    return "";
}

/////////////////////////////
// GET INFO
/////////////////////////////
String MeadeCommandProcessor::handleMeadeGetInfo(String inCmd)
{
    char cmdOne = inCmd[0];
    char cmdTwo = (inCmd.length() > 1) ? inCmd[1] : '\0';

    switch (cmdOne)
    {
        case 'V':
            if (cmdTwo == 'N')  // :GVN
            {
                return String(VERSION) + "#";
            }
            else if (cmdTwo == 'P')  // :GVP
            {
                return "OpenAstroTracker#";
            }
            break;

        case 'X':  // :GX
            return _mount->getStatusString() + "#";

        case 'c':  // :Gc
            {
                return "24#";
            }
        case 'M':  // :GM
            {
                return "OAT1#";
            }
        case 'N':  // :GN
            {
                return "OAT2#";
            }
        case 'O':  // :GO
            {
                return "OAT3#";
            }
        case 'P':  // :GP
            {
                return "OAT4#";
            }
        case 'T':  // :GT
            {
                return "60.0#";  //default MEADE Tracking Frequency
            }
    }

    return "";
}

/////////////////////////////
// MOVEMENT
/////////////////////////////
String MeadeCommandProcessor::handleMeadeMovement(String inCmd)
{
    if (inCmd[0] == 'A')
    {
        LOG(DEBUG_MEADE, "[MEADE]: Move Az/Alt");

        // Move Azimuth or Altitude by given arcminutes
        // :MAZ+32.1# or :MAL-32.1#
        if (inCmd[1] == 'Z')  // :MAZ
        {
            float arcMinute = inCmd.substring(2).toFloat();
            LOG(DEBUG_MEADE, "[MEADE]: Move AZ by %f arcmins", arcMinute);
            _mount->moveBy(AZIMUTH_STEPS, arcMinute);
        }
        if (inCmd[1] == 'L')  // :MAL
        {
            float arcMinute = inCmd.substring(2).toFloat();
            _mount->moveBy(ALTITUDE_STEPS, arcMinute);
        }
        return "";
    }
    else if (inCmd[0] == 'X')  // :MX
    {
        long steps = inCmd.substring(2).toInt();
        LOG(DEBUG_MEADE, "[MEADE]: Move: %l in %d", steps, inCmd[1]);
        if (inCmd[1] == 'z')
            _mount->moveStepperBy(AZIMUTH_STEPS, steps);
        else if (inCmd[1] == 'l')
            _mount->moveStepperBy(ALTITUDE_STEPS, steps);
        else
            return "0";
        return "1";
    }

    return "0";
}

String MeadeCommandProcessor::processCommand(String inCmd)
{
    if (inCmd[0] == ':')
    {
        LOG(DEBUG_MEADE, "[MEADE]: Received command '%s'", inCmd.c_str());

        // Apparently some LX200 implementations put spaces in their commands..... remove them with impunity.
        int spacePos;
        while ((spacePos = inCmd.indexOf(' ')) != -1)
        {
            inCmd.remove(spacePos, 1);
        }

        LOG(DEBUG_MEADE, "[MEADE]: Processing command '%s'", inCmd.c_str());
        char command = inCmd[1];
        inCmd        = inCmd.substring(2);
        switch (command)
        {
            case 'M':
                return handleMeadeMovement(inCmd);
            case 'G':
                return handleMeadeGetInfo(inCmd);
            default:
                LOG(DEBUG_MEADE, "[MEADE]: Received unknown command '%s'", inCmd.c_str());
                break;
        }
    }
    return "";
}
