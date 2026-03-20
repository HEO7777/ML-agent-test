using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine.InputSystem;

public class LegAgent : Agent
{
    // 관절의 ArticulationBody들을 연결
    public ArticulationBody thigh;
    public ArticulationBody shin;

    // 초기 위치/회전 저장을 위한 변수 (에피소드 리셋용)
    private Quaternion initialThighRotation;
    private Quaternion initialShinRotation;
    private Vector3 initialBodyPosition;

    public override void Initialize()
    {
        initialThighRotation = thigh.transform.localRotation;
        initialShinRotation = shin.transform.localRotation;
        initialBodyPosition = transform.localPosition;
    }

    // 1. 에피소드 시작 시 리셋 (로봇이 넘어지면 다시 세워줌)
    public override void OnEpisodeBegin()
    {
        transform.localPosition = initialBodyPosition;
        transform.localRotation = Quaternion.identity;

        // 관절 속도 초기화
        thigh.linearVelocity = Vector3.zero;
        thigh.angularVelocity = Vector3.zero;
        shin.linearVelocity = Vector3.zero;
        shin.angularVelocity = Vector3.zero;

        // 관절 각도(Reduced Space) 리셋
        var zeroJoint = new ArticulationReducedSpace(0f);
        thigh.jointPosition = zeroJoint;
        shin.jointPosition = zeroJoint;
        
        // 관절의 힘(Drive) 목표값도 초기화
        var driveT = thigh.xDrive;
        driveT.target = 0f;
        thigh.xDrive = driveT;

        var driveS = shin.xDrive;
        driveS.target = 0f;
        shin.xDrive = driveS;
    }

    // 2. 환경 관측 (현재 다리 각도, 몸체 높이 등)
    public override void CollectObservations(VectorSensor sensor)
    {
        // 다리 관절의 현재 각도 (정규화해서 넣어주는 게 좋음)
        sensor.AddObservation(thigh.jointPosition[0]); 
        sensor.AddObservation(shin.jointPosition[0]);
        // 몸체의 높이
        sensor.AddObservation(transform.localPosition.y);
    }

    // 3. 행동 수행 (AI가 준 숫자로 관절 움직이기)
    public override void OnActionReceived(ActionBuffers actions)
    {
        // AI가 -1.0 ~ 1.0 사이의 값을 주면, 이를 목표 각도로 변환
        float thighTarget = actions.ContinuousActions[0] * 45f; // 최대 45도
        float shinTarget = actions.ContinuousActions[1] * 45f;

        // Drive의 target 설정
        var thighDrive = thigh.xDrive;
        thighDrive.target = thighTarget;
        thigh.xDrive = thighDrive;

        var shinDrive = shin.xDrive;
        shinDrive.target = shinTarget;
        shin.xDrive = shinDrive;

        // 보상 설계 (예: 몸체 높이 유지)
        if (transform.localPosition.y > 0.5f) {
            AddReward(0.1f);
        }

        // 실패 조건 (몸통이 바닥에 닿으면)
        if (transform.localPosition.y < 0.2f) {
            EndEpisode();
        }
    }

    // 직접 조종하기 위한 코드
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
        
        // 키보드 객체를 가져옵니다.
        var keyboard = Keyboard.current;
        if (keyboard == null) return;

        // 예전 Input.GetAxis("Vertical") 역할을 직접 구현
        float vertical = 0;
        if (keyboard.wKey.isPressed) vertical = 1f;
        else if (keyboard.sKey.isPressed) vertical = -1f;

        float horizontal = 0;
        if (keyboard.dKey.isPressed) horizontal = 1f;
        else if (keyboard.aKey.isPressed) horizontal = -1f;

        continuousActions[0] = vertical;
        continuousActions[1] = horizontal;
    }
}