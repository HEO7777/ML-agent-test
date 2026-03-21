using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine.InputSystem;

public class LegAgent : Agent
{
    // 관절의 ArticulationBody들을 연결
    [Header("--- Articulation Objects ---")]
    public ArticulationBody frontLeftThigh;
    public ArticulationBody frontLeftShin;
    public ArticulationBody frontRightThigh;
    public ArticulationBody frontRightShin;
    public ArticulationBody backLeftThigh;
    public ArticulationBody backLeftShin;
    public ArticulationBody backRightThigh;
    public ArticulationBody backRightShin;

    // 초기 위치/회전 저장을 위한 변수 (에피소드 리셋용)
    private Quaternion flInitialThighRotation;
    private Quaternion flInitialShinRotation;
    private Quaternion frInitialThighRotation;
    private Quaternion frInitialShinRotation;
    private Quaternion blInitialThighRotation;
    private Quaternion blInitialShinRotation;
    private Quaternion brInitialThighRotation;
    private Quaternion brInitialShinRotation;
    private Vector3 initialBodyPosition;
    private Quaternion initialBodyRotation;

    public override void Initialize()
    {
        // 1. 몸통(Root)의 초기 위치 및 회전 저장
        initialBodyPosition = transform.localPosition;
        initialBodyRotation = transform.localRotation;

        // 2. 앞왼쪽 다리 (Front Left)
        flInitialThighRotation = frontLeftThigh.transform.localRotation;
        flInitialShinRotation = frontLeftShin.transform.localRotation;

        // 3. 앞오른쪽 다리 (Front Right)
        frInitialThighRotation = frontRightThigh.transform.localRotation;
        frInitialShinRotation = frontRightShin.transform.localRotation;

        // 4. 뒤왼쪽 다리 (Back Left)
        blInitialThighRotation = backLeftThigh.transform.localRotation;
        blInitialShinRotation = backLeftShin.transform.localRotation;

        // 5. 뒤오른쪽 다리 (Back Right)
        brInitialThighRotation = backRightThigh.transform.localRotation;
        brInitialShinRotation = backRightShin.transform.localRotation;
        
        Debug.Log("Robot Agent Initialized: All joint rotations captured.");
    }

// 1. 에피소드 시작 시 리셋 (로봇이 넘어지면 다시 세워줌)
    public override void OnEpisodeBegin()
    {
        // 1-1. 몸통(Root) 위치 및 회전 리셋
        transform.localPosition = initialBodyPosition;
        transform.localRotation = initialBodyRotation; // Quaternion.identity 대신 저장해둔 초기값 사용

        // 1-2. 각 관절 리셋 (헬퍼 메서드 활용)
        // 앞다리
        ResetJoint(frontLeftThigh, flInitialThighRotation);
        ResetJoint(frontLeftShin, flInitialShinRotation);
        ResetJoint(frontRightThigh, frInitialThighRotation);
        ResetJoint(frontRightShin, frInitialShinRotation);

        // 뒷다리
        ResetJoint(backLeftThigh, blInitialThighRotation);
        ResetJoint(backLeftShin, blInitialShinRotation);
        ResetJoint(backRightThigh, brInitialThighRotation);
        ResetJoint(backRightShin, brInitialShinRotation);
    }

    // 관절 하나를 완벽하게 초기화하는 헬퍼 메서드
    private void ResetJoint(ArticulationBody joint, Quaternion initialRotation)
    {
        // 시각적, 기준점 회전 리셋
        joint.transform.localRotation = initialRotation;

        // 관절 속도 초기화
        joint.linearVelocity = Vector3.zero;
        joint.angularVelocity = Vector3.zero;

        // 관절 각도(Reduced Space) 리셋
        joint.jointPosition = new ArticulationReducedSpace(0f);
        
        // 관절의 힘(Drive) 목표값 초기화
        var drive = joint.xDrive;
        drive.target = 0f;
        joint.xDrive = drive;
    }

    // 2. 환경 관측 (현재 다리 각도, 몸체 높이 등)
    public override void CollectObservations(VectorSensor sensor)
    {
        // 1. 몸체의 높이 (1개)
        sensor.AddObservation(transform.localPosition.y);

        // 2. 앞다리 관절 각도 (4개) (정규화해서 넣어주는 게 좋음)
        sensor.AddObservation(frontLeftThigh.jointPosition[0]); 
        sensor.AddObservation(frontLeftShin.jointPosition[0]);
        sensor.AddObservation(frontRightThigh.jointPosition[0]); 
        sensor.AddObservation(frontRightShin.jointPosition[0]);

        // 3. 뒷다리 관절 각도 (4개) (정규화해서 넣어주는 게 좋음)
        sensor.AddObservation(backLeftThigh.jointPosition[0]); 
        sensor.AddObservation(backLeftShin.jointPosition[0]);
        sensor.AddObservation(backRightThigh.jointPosition[0]); 
        sensor.AddObservation(backRightShin.jointPosition[0]);
    }

    // 3. 행동 수행 (AI가 준 8개의 숫자로 관절 움직이기)
    public override void OnActionReceived(ActionBuffers actions)
    {
        var continuousActions = actions.ContinuousActions;

        // AI가 -1.0 ~ 1.0 사이의 값을 주면, 최대 각도(예: 45도)로 변환해서 적용
        // 앞다리
        ApplyDriveTarget(frontLeftThigh,  continuousActions[0] * 45f);
        ApplyDriveTarget(frontLeftShin,   continuousActions[1] * 45f);
        ApplyDriveTarget(frontRightThigh, continuousActions[2] * 45f);
        ApplyDriveTarget(frontRightShin,  continuousActions[3] * 45f);

        // 뒷다리
        ApplyDriveTarget(backLeftThigh,  continuousActions[4] * 45f);
        ApplyDriveTarget(backLeftShin,   continuousActions[5] * 45f);
        ApplyDriveTarget(backRightThigh, continuousActions[6] * 45f);
        ApplyDriveTarget(backRightShin,  continuousActions[7] * 45f);

        // 1. 전진 속도 보상 (Z축 방향 속도)
        // 로봇의 로컬 Z축 방향 속도가 빠를수록 보상을 줍니다.
        float forwardSpeed = transform.InverseTransformDirection(GetComponent<ArticulationBody>().linearVelocity).z;
        AddReward(forwardSpeed * 0.01f);

        // 2. 몸체 높이 유지
        if (transform.localPosition.y > 0.5f) {
            AddReward(0.01f);
        }

        // 3. 생존 보상 (매 프레임 조금씩 보상을 주어 오래 버티게 함)
        AddReward(0.001f);

        // 실패 조건 (몸통이 바닥에 닿으면 에피소드 종료)
        if (transform.localPosition.y < 0.2f) {
            EndEpisode();
        }
    }

    // 관절을 실제로 움직이는 헬퍼 메서드
    private void ApplyDriveTarget(ArticulationBody joint, float targetAngle)
    {
        var drive = joint.xDrive;
        drive.target = targetAngle;
        joint.xDrive = drive;
    }

    // 4. 수동 조종 테스트 (Stiffness / Damping 조절용)
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
        var keyboard = Keyboard.current;
        if (keyboard == null) return;

        // 배열 초기화 (아무 키도 안 누르면 0 유지)
        for (int i = 0; i < 8; i++) continuousActions[i] = 0f;

        // 테스트 전략: 다리 4개를 그룹으로 묶어서 조종 (안정성 테스트용)
        // W/S 키: 4개의 허벅지(Thigh)를 동시에 앞뒤로 움직임
        float thighMove = keyboard.wKey.isPressed ? 1f : (keyboard.sKey.isPressed ? -1f : 0f);
        
        // A/D 키: 4개의 정강이(Shin)를 동시에 앞뒤로 움직임
        float shinMove = keyboard.aKey.isPressed ? 1f : (keyboard.dKey.isPressed ? -1f : 0f);

        // 허벅지(Thigh) 행동 입력 (인덱스: 0, 2, 4, 6)
        continuousActions[0] = thighMove;
        continuousActions[2] = thighMove;
        continuousActions[4] = thighMove;
        continuousActions[6] = thighMove;

        // 정강이(Shin) 행동 입력 (인덱스: 1, 3, 5, 7)
        continuousActions[1] = shinMove;
        continuousActions[3] = shinMove;
        continuousActions[5] = shinMove;
        continuousActions[7] = shinMove;
    }
}