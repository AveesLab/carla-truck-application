#include <stdio.h>
#include <fstream>
#include "FV1.h"  // FV 모델 헤더

static FV1 rtObj;

// CSV 저장용 ofstream 전역 선언
std::ofstream csvFile;

// Step 함수
void rt_OneStep(int step)
{
    static bool OverrunFlag{ false };
    if (OverrunFlag) {
        rtObj.getRTM()->setErrorStatus("Overrun");
        return;
    }
    OverrunFlag = true;

    // 입력값 갱신 (0.1초마다 2m씩 전방차량 이동, 속도 고정)
    rtObj.rtU.pos_front += 2.0;
    rtObj.rtU.vel_front = 20.8;

    // 모델 스텝 실행
    rtObj.step();

    // 출력값 읽기
    double position = rtObj.rtY.Positionm;
    double speed    = rtObj.rtY.Speedms;
    double soc      = rtObj.rtY.Battery_SOC;
    double throttle = rtObj.rtY.Throttle;
    double brake    = rtObj.rtY.Brake;

    // 콘솔 로그 출력
    printf("[Step %5d] pos_front: %.2f | vel_front: %.2f | Speed: %.2f | Position: %.2f | SOC: %.2f | Throttle: %.2f | Brake: %.2f\n",
           step, rtObj.rtU.pos_front, rtObj.rtU.vel_front, speed, position, soc, throttle, brake);

    // CSV 파일 저장
    csvFile << step << "," << rtObj.rtU.pos_front << "," << rtObj.rtU.vel_front << ","
            << speed << "," << position << "," << soc << "," << throttle << "," << brake << "\n";

    OverrunFlag = false;
}

int main(int argc, const char *argv[])
{
    (void)(argc);
    (void)(argv);

    // 모델 초기화
    rtObj.initialize();
    rtObj.rtU.pos_front = 0.0;
    rtObj.rtU.vel_front = 20.8;

    // CSV 파일 열기
    csvFile.open("fv_output.csv", std::ios::out | std::ios::trunc);
    csvFile << "Step,pos_front,vel_front,Speed,Position,SOC,Throttle,Brake\n";

    // 시뮬레이션 루프
    int step = 0;
    while (rtObj.getRTM()->getErrorStatus() == nullptr &&
           !rtObj.getRTM()->getStopRequested()) {
        rt_OneStep(step++);
    }

    csvFile.close();
    printf("\nSimulation complete. Data saved to fv_output.csv\n");
    return 0;
}
