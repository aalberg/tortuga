#include "../../embedded/sbr5/buscodes.h"

#define MAX_SYNC_ATTEMPTS 20

#define NUM_TEMP_SENSORS 7

struct powerInfo
{
    float motorCurrents[8]; /* Currents for motors and marker droppers */
    float v12VBus;          /* Voltage of 12V bus, in V. */
    float v5VBus;           /* Voltage of 5V bus, in V */
    float i12VBus;          /* Current of 12V bus, in A */
    float i5VBus;           /* Current of 5V bus, in A */
    float iAux;             /* Current of aux (carnetix) output, in A */

    float v26VBus;          /* Voltage of balanced 26V, in V. NOT IMPLEMENTED IN BALANCER r2 */
                            /* Reads as complete garbage */

    float battVoltages[5];  /* 0-3 are batt 1-4. 4 is external power (batt 5). In V */
    float battCurrents[5];  /* Battery currents. See note above. In A */
};

struct boardInfo
{
    int updateState;    /* Internal use only */
    int status;         /* Status register- start switch, kill switch, water sensing */
    int thrusterState;  /* Which thrusters are on */
    int barState;       /* Which bar outputs are on */
    int ovrState;       /* Which thrusters have over-currented */
    int battEnabled;    /* Which batteries are enabled (not the same as in use) */
    int battUsed;       /* Which batteries are being drawn by the balancing circuit */


    struct powerInfo powerInfo;  /* Everything related to power. See above */

    /* Temperatures, in deg C */
    /* These are scattered throughout. The first one is the sensorboard temp. */
    /* The last two are distro and balancer temp (or vice versa?)   */
    /* The middle ones are floaties, if we even have them connected */
    unsigned char temperature[NUM_TEMP_SENSORS];
};


/* In msec */
#define IO_TIMEOUT  100





/* LCD backlight control */
#define LCD_BL_OFF    0
#define LCD_BL_ON     1
#define LCD_BL_FLASH  2

/* Control command return values */
#define SB_OK        0
#define SB_UPDATEDONE 1
#define SB_IOERROR  -4
#define SB_BADCC    -3
#define SB_HWFAIL   -2
#define SB_ERROR    -1


/* Inputs to the thruster safety command */
#define CMD_THRUSTER1_OFF     0
#define CMD_THRUSTER2_OFF     1
#define CMD_THRUSTER3_OFF     2
#define CMD_THRUSTER4_OFF     3
#define CMD_THRUSTER5_OFF     4
#define CMD_THRUSTER6_OFF     5

#define CMD_THRUSTER1_ON      6
#define CMD_THRUSTER2_ON      7
#define CMD_THRUSTER3_ON      8
#define CMD_THRUSTER4_ON      9
#define CMD_THRUSTER5_ON      10
#define CMD_THRUSTER6_ON      11



/* Inputs to the bar command */
#define CMD_BAR1_OFF     0x00
#define CMD_BAR2_OFF     0x01
#define CMD_BAR3_OFF     0x02
#define CMD_BAR4_OFF     0x03
#define CMD_BAR5_OFF     0x04
#define CMD_BAR6_OFF     0x05
#define CMD_BAR7_OFF     0x06
#define CMD_BAR8_OFF     0x07

#define CMD_BAR1_ON    0x08
#define CMD_BAR2_ON    0x09
#define CMD_BAR3_ON    0x0A
#define CMD_BAR4_ON    0x0B
#define CMD_BAR5_ON    0x0C
#define CMD_BAR6_ON    0x0D
#define CMD_BAR7_ON    0x0E
#define CMD_BAR8_ON    0x0F


/* Inputs to the battery control command */
#define CMD_BATT1_OFF     0x00
#define CMD_BATT2_OFF     0x01
#define CMD_BATT3_OFF     0x02
#define CMD_BATT4_OFF     0x03
#define CMD_BATT5_OFF     0x04

#define CMD_BATT1_ON      0x05
#define CMD_BATT2_ON      0x06
#define CMD_BATT3_ON      0x07
#define CMD_BATT4_ON      0x08
#define CMD_BATT5_ON      0x09


/* Bits of the thruster state response */
#define THRUSTER1_ENABLED     0x01
#define THRUSTER2_ENABLED     0x02
#define THRUSTER3_ENABLED     0x04
#define THRUSTER4_ENABLED     0x08
#define THRUSTER5_ENABLED     0x10
#define THRUSTER6_ENABLED     0x20
#define ALL_THRUSTERS_ENABLED \
    (THRUSTER1_ENABLED | THRUSTER2_ENABLED | THRUSTER3_ENABLED | \
     THRUSTER4_ENABLED | THRUSTER5_ENABLED | THRUSTER6_ENABLED)

/* Overcurrent bits. Last 2 are marker droppers */
#define THRUSTER1_OVR     0x01
#define THRUSTER2_OVR     0x02
#define THRUSTER3_OVR     0x04
#define THRUSTER4_OVR     0x08
#define THRUSTER5_OVR     0x10
#define THRUSTER6_OVR     0x20


/* Bits of the bar state response */
#define BAR1_ENABLED    0x01
#define BAR2_ENABLED    0x02
#define BAR3_ENABLED    0x04
#define BAR4_ENABLED    0x08
#define BAR5_ENABLED    0x10
#define BAR6_ENABLED    0x20
#define BAR7_ENABLED    0x40
#define BAR8_ENABLED    0x80


/* Bits of the battery state response */
/* Ie, is the battery enabled? Does not imply the battery is actually in use */
/* For that, see below. */
#define BATT1_ENABLED      0x10
#define BATT2_ENABLED      0x08
#define BATT3_ENABLED      0x04
#define BATT4_ENABLED      0x02
#define BATT5_ENABLED      0x01



/* Bits of the battery utilization response */
/* Ie, is the battery actually being used? */
#define BATT1_INUSE       0x10
#define BATT2_INUSE       0x08
#define BATT3_INUSE       0x04
#define BATT4_INUSE       0x02
#define BATT5_INUSE       0x01


/* Bits of the status response */
/* Water is present */
#define STATUS_WATER      0x20

/* Kill switch is attached */
#define STATUS_KILLSW     0x40

/* Start switch is being pressed */
#define STATUS_STARTSW    0x80



// If we are compiling as C++ code we need to use extern "C" linkage
#ifdef __cplusplus
extern "C" {
#endif // __cplusplus


/* Perform next step of update cycle.
 * Returns: SB_OK on success
 *          SB_UPDATEDONE on success and update cycle is done
 *          SB_ERROR, SB_IOERROR, SB_BADCC, SB_HWFAIL, SB_ERROR on failure
 */
int partialRead(int fd, struct boardInfo * info);

/** Returns the file*/
int openSensorBoard(const char * devName);

/** Syncs the communication protocol between the board and vehicle */
int syncBoard(int fd);

int checkBoard(int fd);


int pingBoard(int fd);

/** Requests the depth value from the device (or error code)
 *
 *  @return An integer between 0 and 1023, or SB_ERROR.
 */
int readDepth(int fd);

/** Read the status bit back from the board */
int readStatus(int fd);

/** Reads the state of thrusters (safed or not)
 *  Returns a bit combination of THRUSTERx_ENABLED as above
 *  or SB_ERROR. How to tell them apart? SB_ERROR is negative,
 *  don't worry.
 */
int readThrusterState(int fd);

int hardKill(int fd);

/** This doesn't do anything anymore. Return value is undefined.
 * Marker dropper support to return in the next revision hopefully.
 */
int dropMarker(int fd, int markerNum);

int lcdBacklight(int fd, int state);

/** Either enables or disables a desired thruster */
int setThrusterSafety(int fd, int state);

int setBarState(int fd, int state);

int displayText(int fd, int line, const char* text);

/**  Reads the values from the board's temperature

     @param fd
         The file descriptor returned by openSensorBoard()
     @param tempData
         Where the sensor data is written. The array must be at least
         NUM_TEMP_SENSORS elements long. The temperatures are specified in
         degrees C. A value of 255 indicates a missing or malfunctioning
         sensor.
     @return SB_OK upon success or SB_ERROR.
**/
int readTemp(int fd, unsigned char * tempData);

int getSonarData(int fd, int * angle, int * distance, int * pingNumber);

int setDiagnostics(int fd, int state);

/** Set the speed of the thrusters

    This command takes about 2 ms to execute.  You must call
    readSpeedResponses before this command, or about 15 ms after this call is
    made.

    @param fd
         The file descriptor returned by openSensorBoard()

    @param s1
         The speed of thruster with address one
    @param s2
         The speed of thruster with address two
    @param s3
         The speed of thruster with address three
    @param s4
         The speed of thruster with address four
    @param s5
         The speed of thruster with address five
    @param s6
         The speed of thruster with address six
 */
int setSpeeds(int fd, int s1, int s2, int s3, int s4, int s5, int s6);

/** Reads back the on the board from the motor controller

    This is basically a house cleaning command, seee setSpeeds for information
    on its use.
 */
int readSpeedResponses(int fd);

int readThrusterState(int fd);

int readBarState(int fd);

int readOvrState(int fd);

int readBatteryEnables(int fd);

int readBatteryUsage(int fd);

int readMotorCurrents(int fd, struct powerInfo * info);
int readBoardVoltages(int fd, struct powerInfo * info);

int readBatteryVoltages(int fd, struct powerInfo * info);
int readBatteryCurrents(int fd, struct powerInfo * info);

int switchToExternalPower(int fd);
int switchToInternalPower(int fd);

int setBatteryState(int fd, int state);


// maxCurrent (mA) = (a * speed) / 6 + b*40
// where speed=[0,255] corresponds to 0 to full speed
int setOvrParams(int fd, int a, int b);
int readOvrParams(int fd, int * a, int * b);

// If we are compiling as C++ code we need to use extern "C" linkage
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus
