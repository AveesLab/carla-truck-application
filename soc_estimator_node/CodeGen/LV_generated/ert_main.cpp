#include <stdio.h>
#include <fstream>
#include "LV.h"

static LV rtObj;

void rt_OneStep(std::ofstream& csvFile, int step, bool& socBelowThreshold);
void rt_OneStep(std::ofstream& csvFile, int step, bool& socBelowThreshold)
{
    static bool OverrunFlag{ false };
    if (OverrunFlag) {
        rtObj.getRTM()->setErrorStatus("Overrun");
        return;
    }

    OverrunFlag = true;

    // 위치 기반 pos_follow 입력 업데이트
    double position = rtObj.rtY.Positionm;
    if (position > 12.0) {
        rtObj.rtU.pos_follow = position - 12.0;
    } else {
        rtObj.rtU.pos_follow = 0.0;
    }

    // 모델 스텝 실행
    rtObj.step();

    // 입력 및 출력
    double ts = rtObj.rtU.Targetspeed;
    double pos = rtObj.rtU.pos_follow;
    double speed = rtObj.rtY.Speedms;
    double soc = rtObj.rtY.Battery_SOC;

    // SOC 종료 조건 검사
    if (soc <= 20.0) {
        socBelowThreshold = true;
    }

    // 콘솔 출력
    printf("[Step %d] TargetSpeed: %.2f | pos_follow: %.2f | Speed: %.2f | Position: %.2f | SOC: %.2f\n",
           step, ts, pos, speed, position, soc);

    // CSV 저장
    csvFile << step << "," << ts << "," << pos << "," << speed << "," << position << "," << soc << "\n";

    OverrunFlag = false;
}

int main(int argc, const char *argv[])
{
    (void)(argc);
    (void)(argv);

    std::ofstream csvFile("SOC_output.csv");
    csvFile << "Step,TargetSpeed,pos_follow,Speed,Position,SOC\n";

    // 모델 초기화
    rtObj.initialize();

    // 초기 입력값
    rtObj.rtU.Targetspeed = 0.0;
    rtObj.rtU.pos_follow = 0.0;

    int step = 0;
    bool socBelowThreshold = false;

    while (rtObj.getRTM()->getErrorStatus() == nullptr &&
           !rtObj.getRTM()->getStopRequested() &&
           !socBelowThreshold) {
        rt_OneStep(csvFile, step++, socBelowThreshold);
    }

    csvFile.close();
    printf("\nSimulation complete. Data saved to SOC_output.csv\n");
    return 0;
}
