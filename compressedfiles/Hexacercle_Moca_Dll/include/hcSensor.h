#pragma once
#include "hcQuaternion.h"
#include "Filter.h"
#include "stdint.h"
namespace Hexacercle
{

   

    namespace SensorTypes
    {
        const int Unknown = 0;
        const int All = 0xff;
        // ����
        // ͷ
        const int Head = 0x04;
        // ��
        const int Chest = 0x05;
        // ��
        const int Abdomen = 0x06;

        //�Ҳ�
        // ������
        const int RightGlove = 0x10;
        // ����ͨѶЭ���У� �����ڱ�ʶ
        const int RightThumb = 0x110;
        //// ������
        //const int RightWaist = 0x11;
        // ������
        const int RightElbow = 0x12;
        // �Ҽ��
        const int RightShoulder = 0x13;
        // ���Ų�
        const int RightHip = 0x17;
        // ��ϥ��
        const int RightKnee = 0x18;
        // �ҽ���
        const int RightAnkle = 0x19;

        //���
        // ������
        const int LeftGlove = 0x20;
        // ����ͨѶЭ���У� �����ڱ�ʶ
        const int LeftThumb = 0x220;
        //// ����
        //const int LeftWaist = 0x21;
        // ����
        const int LeftElbow = 0x22;
        // ���
        const int LeftShoulder = 0x23;
        // ���Ų�
        const int LeftHip = 0x27;
        // ��ϥ��
        const int LeftKnee = 0x28;
        // �����
        const int LeftAnkle = 0x29;

        const int Gun = 0xB4;

        // ����ģʽ
        const int Configuration = 0x3F;

        const int TypeEnum[] = {
        Head, Chest, Abdomen, Gun,
        RightGlove, RightElbow, RightShoulder, RightHip, RightKnee, RightAnkle, 
        LeftGlove, LeftElbow, LeftShoulder, LeftHip, LeftKnee, LeftAnkle};
    }

    class hcSensor
    {
    public:
        int sType;
        float Voltage;
        float Voltpercent;
        hcSensor():
            sType(SensorTypes::Unknown),
            Voltage(0),
            Voltpercent(0)
        {}
        hcSensor(const hcSensor& other)
        {
            sType = other.sType;
            Voltage = other.Voltage;
            Voltpercent = other.Voltpercent;
        }
        virtual ~hcSensor() {};
    };

    class MocaSensor : public hcSensor
    {
    public:
        bool TransInitialized;
        Quaternion QuatTrans;//�����õ�����
        Quaternion QuatPitch;//��������ԭʼ����
        float Alpha, Beta;
        Param param;
        MocaSensor():
            TransInitialized(false),
            QuatTrans(),
            QuatPitch(),
            Alpha(0), Beta(0),
            param()
        {}
        MocaSensor(const MocaSensor& other):
            hcSensor(other)
        {
            TransInitialized = other.TransInitialized;
            QuatTrans = other.QuatTrans;
            QuatPitch = other.QuatPitch;
            Alpha = other.Alpha; Beta = other.Beta;
            param = other.param;
        }
    };

    class AccSensor : public MocaSensor
    {
    public:
        float Acc[3];
        AccSensor():
            Acc{0}
        {}
        AccSensor(const AccSensor &other):
            MocaSensor(other)
        {
            for(int i = 0 ; i < 3 ; i ++)
                Acc[i] = other.Acc[i];
        }
    };

    class AnkleSensor : public MocaSensor
    {
    public:
        uint8_t FrontPressure, RearPressure;
        float Acc[3];
        AnkleSensor() :
            FrontPressure(0xff),
            RearPressure(0xff),
            Acc{0}
        {}
        AnkleSensor(const AnkleSensor& other) :
            MocaSensor(other)
        {
            for (int i = 0; i < 3; i++)
                Acc[i] = other.Acc[i];
        }
    };

    class GloveSensor : public MocaSensor
    {
    public:
        bool FingerInitialized;
        Quaternion FingerTrans;
        Quaternion FingerPitch[5];
        float FingerAlpha[6];
        float FingerBeta[6];
        Param FingerParam[6];
        GloveSensor() :
            FingerInitialized(false),
            FingerAlpha{0,0,0,0,0},
            FingerBeta{ 0,0,0,0,0 }
        {};
        GloveSensor(const GloveSensor& other):
            MocaSensor(other),
            FingerInitialized(other.FingerInitialized),
            FingerTrans(other.FingerTrans)
        {
            for (int i = 0; i < 5; i++)
            {
                FingerPitch[i] = other.FingerPitch[i];
            }
            for (int i = 0; i < 6; i++)
            {
                FingerAlpha[i] = other.FingerAlpha[i];
                FingerBeta[i] = other.FingerBeta[i];
                FingerParam[i] = other.FingerParam[i];
            }
        }
    };

    namespace GunKeys
    {
        // ǹ�������� �ఴ��ͬʱ����ʱ��And��ֵ

        // �����
        const uint8_t Trigger = 0x01;
        // ���ţ���ǹ˨�� Vive�����
        const uint8_t Recharge = 0x02;
        // ����ϻ�� Vive�˵���
        const uint8_t Reload = 0x04;
        // Viveϵͳ��
        const uint8_t System = 0x08;
        // ������İ�;����; ���ڷ�ֹ��������
        const uint8_t PreTrigger = 0x10;
        // ����ǹ˨�İ�;����; ���ڷ�ֹ��������
        const uint8_t PreSlider = 0x20;
    }

    struct GunState
    {
        bool Enabled;
        int GunID;
        uint8_t GunType;
        uint8_t GunKeys;
        float axisX;
        float axisY;
        float Trigger;
        float Battery;

        GunState() :
            Enabled(false),
            GunID(0),
            GunType(0),
            GunKeys(0),
            axisX(0),
            axisY(0),
            Trigger(0),
            Battery(0)
        {}
    };

    class GunSensor : public hcSensor
    {
    public:
        GunState State;
    };
}