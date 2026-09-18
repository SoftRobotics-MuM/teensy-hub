// Ported from sensor-pcb-v2-master-test-software (SoftRobotics-MuM)
// teensy_test_software.ino - ping/mag/IMU-relay logic, adapted from an
// interactive serial test menu into periodic polling for read_sensor_board(),
// and from default hardware SPI0 to SPI1 (see sensor_cfg.h).
#include "sensor_pcb_v2.h"
#include "sensor_cfg.h"
#include "Protocol.h"
#include "Lsm6dsv16bx.h"
#include <Arduino.h>
#include <SPI.h>
#include <string.h>

namespace {

SPISettings busSettings(SPI_BUS_HZ, MSBFIRST, SPI_MODE0);

// Phase A: the 4-byte command frame every STM32 command starts with (see
// Protocol.h's two-phase transaction model). Blocks for T_PROC_GUARD_US
// afterward so the STM32's ISR has time to act before anything else touches
// the bus (a following Phase B, or a direct IMU transaction).
void sendPhaseA(uint8_t cmd, uint8_t targetAddr, uint8_t d0 = 0x00, uint8_t d1 = 0x00) {
    uint8_t txBuf[4] = { cmd, targetAddr, d0, d1 };
    SPI1.beginTransaction(busSettings);
    digitalWrite(PIN_BUS_CS, LOW);
    delayMicroseconds(T_CS_SETUP_GUARD_US);
    SPI1.transfer(txBuf, 4);
    digitalWrite(PIN_BUS_CS, HIGH);
    SPI1.endTransaction();
    delayMicroseconds(T_PROC_GUARD_US);
}

void setImuCs(uint8_t targetAddr, uint8_t csSelect) {
    sendPhaseA(CMD_SET_IMU_CS, targetAddr, csSelect);
}

// Bridges Lsm6dsv16bx's per-register SelectFn callback to this project's
// relay-CS scheme - see Lsm6dsv16bx.h's chip-select model note.
struct ImuCsCtx {
    uint8_t targetAddr;
    uint8_t imuSelect;
};

void imuRelaySelect(void *ctx, bool select) {
    ImuCsCtx *c = (ImuCsCtx *)ctx;
    setImuCs(c->targetAddr, select ? c->imuSelect : IMU_CS_NONE);
}

// Tracks, per board+IMU, whether we've already successfully configured and
// identity-verified it (readRaw() vs the cheaper readData() - see
// Lsm6dsv16bx.h). Indexed [targetAddr][imuSelect]; index 0 (IMU_CS_NONE) is
// unused. Cleared only by restarting the Teensy - see the equivalent note in
// the upstream master test harness for why that's intentional.
bool imuVerified[MAX_SENSOR_BOARDS][3] = {};

void readImu(uint8_t targetAddr, uint8_t imuSelect, Lsm6dsv16bx::Raw &out, bool &valid) {
    ImuCsCtx ctx{ targetAddr, imuSelect };
    if (imuVerified[targetAddr][imuSelect]) {
        Lsm6dsv16bx::readData(imuRelaySelect, &ctx, out);
        valid = true;
    } else {
        valid = Lsm6dsv16bx::readRaw(imuRelaySelect, &ctx, out);
        imuVerified[targetAddr][imuSelect] = valid;
    }
}

// Raw LSB counts -> physical SI units (see sensor_cfg.h). Order matches
// SensorBoard.msg's imu1/imu2 layout: gx,gy,gz,ax,ay,az.
void fillImuField(float *dst, const Lsm6dsv16bx::Raw &raw) {
    dst[0] = raw.gx * GYRO_LSB_TO_MDPS * MDPS_TO_RADPS;
    dst[1] = raw.gy * GYRO_LSB_TO_MDPS * MDPS_TO_RADPS;
    dst[2] = raw.gz * GYRO_LSB_TO_MDPS * MDPS_TO_RADPS;
    dst[3] = raw.ax * ACCEL_LSB_TO_MG * MG_TO_MPS2;
    dst[4] = raw.ay * ACCEL_LSB_TO_MG * MG_TO_MPS2;
    dst[5] = raw.az * ACCEL_LSB_TO_MG * MG_TO_MPS2;
}

// Reconstructs one 20-bit signed-centered raw magnetometer axis from the
// CMD_GET_MAG response's interleaved register layout - see Protocol.h.
int32_t unpackMagAxis(uint8_t hi, uint8_t mid, uint8_t lowNibbleSrc) {
    int32_t v = ((int32_t)hi << 12) | ((int32_t)mid << 4) | (lowNibbleSrc & 0x0F);
    v -= (int32_t)1 << 19;  // MMC5633NJL's 20-bit output is centered at 2^19, not signed two's complement
    return v;
}

bool pingNode(uint8_t targetAddr) {
    sendPhaseA(CMD_PING, targetAddr);

    uint8_t rxBuf[PING_RESP_LEN] = {0};
    SPI1.beginTransaction(busSettings);
    digitalWrite(PIN_BUS_CS, LOW);
    delayMicroseconds(T_CS_SETUP_GUARD_US);
    SPI1.transfer(rxBuf, PING_RESP_LEN);
    digitalWrite(PIN_BUS_CS, HIGH);
    SPI1.endTransaction();
    delayMicroseconds(T_CS_SETUP_GUARD_US);

    return (rxBuf[0] == 0xA5 && rxBuf[1] == FW_VERSION);
}

}  // namespace

void setup_sensor_pcb_v2() {
    pinMode(PIN_BUS_CS, OUTPUT);
    digitalWrite(PIN_BUS_CS, HIGH);  // SPI deselected

    SPI1.setMOSI(PIN_BUS_MOSI);
    SPI1.setMISO(PIN_BUS_MISO);
    SPI1.setSCK(PIN_BUS_SCK);
    SPI1.begin();

    memset(imuVerified, 0, sizeof(imuVerified));

    // Panic release: every board releases both its IMU CS lines, regardless
    // of what state a previous run (or a board reset independent of this
    // sketch) left them in. Always safe - see Protocol.h's CMD_SET_IMU_CS.
    setImuCs(ADDR_BROADCAST, IMU_CS_NONE);

    for (uint8_t addr = 0; addr < MAX_SENSOR_BOARDS; addr++) {
        if (!BOARD_ENABLED[addr]) {
            continue;
        }
        bool ok = pingNode(addr);
        Serial.printf("[sensor_pcb_v2] board %u: %s\n", addr, ok ? "OK" : "NOT RESPONDING (check wiring/address)");
    }
}

void read_sensor_board(uint8_t targetAddr, soro_msgs__msg__SensorBoard &board) {
    board.status = 0;

    // --- IMU1 / IMU2: direct SPI access, CS relayed through the STM32 ---
    Lsm6dsv16bx::Raw raw1, raw2;
    bool imu1Valid = false, imu2Valid = false;
    readImu(targetAddr, IMU_CS_IMU1, raw1, imu1Valid);
    readImu(targetAddr, IMU_CS_IMU2, raw2, imu2Valid);

    if (imu1Valid) {
        fillImuField(board.imu1, raw1);
        board.status |= soro_msgs__msg__SensorBoard__STATUS_IMU1_VALID;
    }
    if (imu2Valid) {
        fillImuField(board.imu2, raw2);
        board.status |= soro_msgs__msg__SensorBoard__STATUS_IMU2_VALID;
    }

    // --- Mag1 / Mag2: STM32 command channel (CMD_GET_MAG) ---
    sendPhaseA(CMD_GET_MAG, targetAddr);

    uint8_t rxBuf[MAG_RESP_LEN] = {0};
    SPI1.beginTransaction(busSettings);
    digitalWrite(PIN_BUS_CS, LOW);
    delayMicroseconds(T_CS_SETUP_GUARD_US);
    SPI1.transfer(rxBuf, MAG_RESP_LEN);
    digitalWrite(PIN_BUS_CS, HIGH);
    SPI1.endTransaction();
    delayMicroseconds(T_CS_SETUP_GUARD_US);

    uint8_t magStatus = rxBuf[0];
    board.seq = rxBuf[19];

    if (magStatus & 0x01) {
        int32_t x = unpackMagAxis(rxBuf[1], rxBuf[2], rxBuf[7]);
        int32_t y = unpackMagAxis(rxBuf[3], rxBuf[4], rxBuf[8]);
        int32_t z = unpackMagAxis(rxBuf[5], rxBuf[6], rxBuf[9]);
        board.mag1[0] = x * MAG_LSB_TO_TESLA;
        board.mag1[1] = y * MAG_LSB_TO_TESLA;
        board.mag1[2] = z * MAG_LSB_TO_TESLA;
        board.status |= soro_msgs__msg__SensorBoard__STATUS_MAG1_VALID;
    }
    if (magStatus & 0x02) {
        int32_t x = unpackMagAxis(rxBuf[10], rxBuf[11], rxBuf[16]);
        int32_t y = unpackMagAxis(rxBuf[12], rxBuf[13], rxBuf[17]);
        int32_t z = unpackMagAxis(rxBuf[14], rxBuf[15], rxBuf[18]);
        board.mag2[0] = x * MAG_LSB_TO_TESLA;
        board.mag2[1] = y * MAG_LSB_TO_TESLA;
        board.mag2[2] = z * MAG_LSB_TO_TESLA;
        board.status |= soro_msgs__msg__SensorBoard__STATUS_MAG2_VALID;
    }
}
