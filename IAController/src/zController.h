/*
* Author: Raghavasimhan Sankaranarayanan
* Date: 05/20/24
*/


#pragma once

#include <CAN.h>

#include "../include/definitions.h"
#include "../include/ErrorDef.h"
#include "logger.h"
#include "epos4/epos4.h"

#include "networkHandler.h"
#include "pid.h"


class ZController {
public:
    static ZController* create() {
        pInstance = new ZController();
        return pInstance;
    }

    static void destroy(ZController* pInstance) {
        delete pInstance;
    }

    // Delete copy and assignment constructor
    ZController(ZController&) = delete;
    void operator=(const ZController&) = delete;

    Error_t init() {
        Error_t err;
        err = initCAN();
        if (err != kNoError)
            return err;

        err = m_socket.init(&ZController::packetCallback);
        if (err != kNoError)
            return err;

#ifndef SIMULATE
        int e = m_epos.init(1, ENCODER_TICKS_PER_TURN[0]);
        if (err != 0) {
            LOG_ERROR("Error initializing motor");
            reset();
            return kNotInitializedError;
        }
#endif
        m_trajTimer.setPeriod(PDO_RATE * 1000);
        m_trajTimer.attachInterrupt(&ZController::RPDOTimerIRQHandler);
        home();

        LOG_LOG("ZController Initialized");
        return kNoError;
    }

    void reset() {
        m_trajTimer.stop();

#ifndef SIMULATE
        m_epos.reset();
#endif
        m_socket.close();

        m_bInitialized = false;
        delay(100);
    }

    Error_t initCAN() {
        if (!CanBus.begin(CAN_BAUD_1000K, CAN_STD_FORMAT)) {
            LOG_ERROR("CAN open failed");
            return kConfigError;
        }

        CanBus.attachRxInterrupt(&ZController::canRxHandle);
        return kNoError;
    }

    void poll() {
        m_socket.poll();
    }

private:
    // An UdpHandler instance to let us receive packets over UDP
    NetworkHandler m_socket;
    HardwareTimer m_trajTimer;

    static ZController* pInstance;

    bool m_bInitialized = false;

    PID pid;
    Epos4 m_epos;

    float m_iCurrentPoint = 0;
    float m_iSetPoint = 0;

    ZController() : m_trajTimer(TIMER_CH3), pid(Kp, Kd, Ki, PDO_RATE) {}

    ~ZController() {
        reset();
    }

    static void packetCallback(char* packet, size_t packetSize) {
        float point = 0;
        if (packetSize == 1) {
            // Commands
        } else if (packetSize == sizeof(point)) {
            // int32_t point = 0;
            // for (int i = 0; i < NUM_BYTES_PER_VALUE; ++i)
            //     point = point | ((packet[i] & 0xFF) << (8 * i));
            memcpy(&point, packet, sizeof(point));
            pInstance->m_iSetPoint = point;
        } else {
            LOG_ERROR("Packet Corrupted. Received %i bytes", packetSize);
        }
    }

    static void RPDOTimerIRQHandler() {
        if (!pInstance->m_bInitialized)
            return;
        float sp = pInstance->m_iSetPoint;
        float cp = pInstance->m_iCurrentPoint;
        float point = pInstance->pid.step(cp, sp, POSITION_OFFSET);
        point = max(point, MIN_POSITION_MM);
        point = min(point, MAX_POSITION_MM);
        int32_t ticks = round(point * ENCODER_TICKS_PER_TURN[0] * 4 / (2 * PI * GT2_PULLEY_RADIUS));

#ifndef SIMULATE
        pInstance->m_epos.PDO_setPosition(ticks);
#endif

        pInstance->m_iCurrentPoint = point;
        sp = pInstance->m_iSetPoint;
        cp = pInstance->m_iCurrentPoint;
        if (abs(sp - cp) > 0.001) {
            Serial.print(cp);
            Serial.print("\t");
            Serial.println(sp);
        }
    }

    static void canRxHandle(can_message_t* arg) {
        bool validId = false;
        auto id = arg->id - COB_ID_SDO_SC;
        if (id == 1) {
            pInstance->m_epos.setRxMsg(*arg);
            validId = true;
        }

        id = arg->id - COB_ID_TPDO3;
        if (id == 1) {
            pInstance->m_epos.PDO_processMsg(*arg);
            validId = true;
        }

        id = arg->id - COB_ID_EMCY;
        if (id == 1) {
            pInstance->m_epos.handleEMCYMsg(*arg);
            validId = true;
        }
    }

    Error_t home() {
        int err;

        m_trajTimer.stop();
#ifndef SIMULATE
        err = m_epos.home(true);
        if (err != 0) {
            LOG_ERROR("home");
            return kSetValueError;
        }

        delay(10);

        int32_t pos;
        err = m_epos.getActualPosition(&pos);
        LOG_LOG("position: %i", (int) pos);
        m_iCurrentPoint = pos;

        LOG_LOG("Homing Complete...");
#else
        delay(1000);
#endif

        auto e = enablePDO();
        if (e != kNoError) return e;

        m_bInitialized = true;
        m_trajTimer.start();

        return kNoError;
    }

    Error_t enable(bool bEnable = true) {
#ifndef SIMULATE
        if (!bEnable) {
            if (m_epos.setNMTState(PreOperational)) {
                LOG_ERROR("setNMTState");
                return kSetValueError;
            }
        }
        if (m_epos.setEnable(bEnable)) {
            LOG_ERROR("setEnable");
            return kSetValueError;
        }
#endif
        return kNoError;
    }

    Error_t enablePDO(bool bEnable = true) {
#ifndef SIMULATE
        auto state = bEnable ? NMTState::Operational : NMTState::PreOperational;
        if (m_epos.setNMTState(state)) {
            LOG_ERROR("setNMTState");
            return kSetValueError;
        }

        if (m_epos.setOpMode(OP_MODE[0])) {
            LOG_ERROR("setOpMode");
            return kSetValueError;
        }
#endif
        return kNoError;
    }
};

ZController* ZController::pInstance = nullptr;