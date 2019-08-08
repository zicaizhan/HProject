using UnityEngine;
using System.Collections;
using HedgehogTeam.EasyTouch;

namespace HRacing
{
    public class SmoothFollowCamera2 : BaseCamera
    {
        GameObject targetGO;
        Transform targetTransform;
        HCar targetSkateboard;
        BasePhysics targetPhysics { get { return targetSkateboard.GetPhysics(); } }
        FollowCameraConfig2 followCameraConfig;
        bool logCameData = false;
        public void Init(GameObject chasedGO)
        {
            BaseInit(chasedGO);
            followCameraConfig = ConfigFileManager.GetInstance().GetConfigDefine<FollowCameraConfig2>(1);
            
            //配置参数
            mCamera.fieldOfView = followCameraConfig.fov;
            mCamera.orthographic = false;
            mCamera.farClipPlane = followCameraConfig.viewDistance;
            this.targetGO = chasedGO;
            targetRigidibody = chasedGO.GetComponent<Rigidbody>();
            targetTransform = chasedGO.transform;
            targetSkateboard = chasedGO.GetComponent<HRacing.HCar>();
            HRacing.InputManager.GetInstance().RegisterAction(HRacing.InputType.Swipe, OnSwipe);
            HRacing.InputManager.GetInstance().RegisterAction(HRacing.InputType.SwipeStart, OnSwipeStart);
            HRacing.InputManager.GetInstance().RegisterAction(HRacing.InputType.SwipeEnd, OnSwipeEnd);
            MessageDispatcher.AddListener(MessageDefine.StartFreeLook, OnStartFreeLook);
            MessageDispatcher.AddListener(MessageDefine.EndFreeLook, OnEndFreeLook);
            MessageDispatcher.AddListener(MessageDefine.FreeLooking, OnFreeLooking);
            if (logCameData)
                LogManager.GetInstance().LogNoTime("车身和速度方向夹角,摄像机和车身夹角,车身角度, 跟随速度, 帧时间,摄像机目标与车身");
        }
        float smoothForwardAccel = 0;
        Quaternion smoothTargetRotation = Quaternion.identity;
        float smoothAddDistance = 0;
        float smoothTilitAngle = 0;
        //Quaternion wantedRotation = Quaternion.identity;
        float followSpeed = 0;
        float smoothYAddHeight = 0;
        protected override void UpdateCamera(float frameTime)
        {
            if (targetRigidibody == null )
                return;
            //             if (targetPhysics == null)
            //                 return;
            float followDistance;
            float followAngle;
            float followOffsetHeight;
            int curFlightStatus = targetSkateboard.GetPlayer().FlightStatus;
            if (curFlightStatus == FlightStatusConst.InPlane)
            {
                followDistance = followCameraConfig.followDistanceFlight;
                followAngle = followCameraConfig.followAngleFlight;
                followOffsetHeight = followCameraConfig.followOffsetHeightFlight;
            }
            else if(curFlightStatus == FlightStatusConst.Normal)
            {
                followDistance = followCameraConfig.followDistance;
                followAngle = followCameraConfig.followAngle;
                followOffsetHeight = followCameraConfig.followOffsetHeight;
            }
            else
            {
                followDistance = followCameraConfig.followDistanceFalling;
                followAngle = followCameraConfig.followAngleFalling;
                followOffsetHeight = followCameraConfig.followOffsetHeightFalling;
            }

            float damping = followCameraConfig.followPositionDamping;
            float deltaTime = frameTime;
            const float rotationDamping =2;
            //根据速度和加速度、氮气计算followDistance增加值
            if(targetGO.activeInHierarchy)
            {
                smoothForwardAccel = Mathf.Lerp(smoothForwardAccel, targetPhysics.GetForwardAccel(), deltaTime * followCameraConfig.AccelDamp);
                //速度影响
                float speedn = targetPhysics.GetForwardSpeed() ;
                if (speedn < 0)
                    speedn = 0;
                speedn = Mathf.Pow(speedn, followCameraConfig.B)*followCameraConfig.A;
                float addFollowDistance = 0;
                
                addFollowDistance += Mathf.Clamp( speedn, 0, followCameraConfig.MaxSpeedDistance);
               /* Debug.Log("速度增加距离:" + addFollowDistance);*/
                //氮气影响
                if (targetSkateboard.GetBuf().IsUsingN2()) 
                {
                    //float accel2 = smoothForwardAccel * smoothForwardAccel;
                    addFollowDistance += followCameraConfig.C;
                }
                smoothAddDistance = Mathf.Lerp(smoothAddDistance, addFollowDistance, deltaTime*6f);
                /*Debug.Log("增加总距离:" + smoothAddDistance);*/
                followDistance += smoothAddDistance;
            }
            Vector3 targetPosition = targetTransform.position;

            Vector3 camLocalDir = targetTransform.InverseTransformVector(mTransform.forward);
            camLocalDir.y = 0;
            camLocalDir.Normalize();

            //Debug.Log("摄像机和车身夹角:" + Mathf.Asin(Mathf.Abs(camLocalDir.x)) * Mathf.Rad2Deg);

            float a = 60;
            float b = 10;
			float c = 1000;
            float max = 50;

            Vector3 speedLocalDirTemp = targetTransform.InverseTransformVector(targetRigidibody.velocity);
            speedLocalDirTemp.y = 0;
            speedLocalDirTemp.Normalize();
            float x2 = Mathf.Asin(Mathf.Abs(speedLocalDirTemp.x)) * Mathf.Rad2Deg;
            //Debug.Log("车身和速度方向夹角：" + x2);
            string logMsg="";
            if(logCameData)
            {
                logMsg = x2.ToString() + "," + (Mathf.Asin(Mathf.Abs(camLocalDir.x)) * Mathf.Rad2Deg).ToString() + "," + MathUtility.vectorToXAngle(targetTransform.forward);
                
            }
            x2 = x2 * x2 * x2;   

            followSpeed += Mathf.Abs(speedLocalDirTemp.x/*camLocalDir.x*/) * deltaTime * a * (x2 / (x2 + c));

            followSpeed -= deltaTime * b;
            followSpeed = Mathf.Clamp(followSpeed, 0, max);
           // Debug.Log("followSpeed:" + followSpeed);
           if(logCameData)
            {
                logMsg += "," + followSpeed.ToString() + "," + deltaTime;
            }


            smoothTargetRotation = Quaternion.Slerp(smoothTargetRotation, targetTransform.rotation, (/*followSpeed+ */followCameraConfig.rotationDamping) * deltaTime);
           // smoothTargetRotation = Quaternion.Slerp(smoothTargetRotation, Quaternion.LookRotation(targetRigidibody.velocity, targetRigidibody.transform.up), 1/*rotationDamping * deltaTime*/);

            Vector3 targetUp = smoothTargetRotation * Vector3.up;
            Vector3 targetForward = smoothTargetRotation * Vector3.forward;
            float YSpeedHeight = 0;
            {

                float ySpeed = targetRigidibody.velocity.y;
                float signYSpeed = Mathf.Sign(ySpeed);
                const float n = 1;
                const float A = 0.2f;
                const float maxYOffset = 2;
                YSpeedHeight = Mathf.Pow(ySpeed * signYSpeed, n)*A;

                if (YSpeedHeight > maxYOffset)
                    YSpeedHeight = maxYOffset;
                if (signYSpeed < 0)
                    YSpeedHeight *= signYSpeed * -1;
                else
                    YSpeedHeight = 0;
                smoothYAddHeight = Mathf.Lerp(smoothYAddHeight, YSpeedHeight, frameTime * 2);
            }


            Vector3 wantedPosition = targetPosition + targetForward * (-followDistance) + targetUp * (Mathf.Tan(followAngle * Mathf.Deg2Rad) * followDistance + followOffsetHeight );
            //wantedPosition.y +=  YSpeedHeight;

            mTransform.position = wantedPosition;//Vector3.Lerp(mTransform.position, wantedPosition, deltaTime * damping);
            Vector3 lookAtPosition = targetPosition + targetUp * followOffsetHeight;

            Quaternion wantedRotation = Quaternion.LookRotation(lookAtPosition - mTransform.position, targetUp);

            //倾侧角计算
//             float t3 = targetPhysics.GetSidewaySpeed() * followCameraConfig.TilitA;
//             float signT3 = Mathf.Sign(t3);
//             t3 *= signT3;
//             t3 = t3 * t3 * t3;
//             float tilitAngle = followCameraConfig.MaxTilitAngle * (t3 / (t3 + followCameraConfig.TilitB)) * signT3;
//             smoothTilitAngle = Mathf.Lerp(smoothTilitAngle, tilitAngle, deltaTime * 10);
            //Quaternion tilitRot = Quaternion.AngleAxis(tilitAngle, Vector3.forward);
            //mTransform.Rotate(Vector3.forward, tilitAngle, Space.Self);
           // Debug.Log(tilitAngle);
            mTransform.rotation = wantedRotation;// * tilitRot;// Quaternion.Slerp(mTransform.rotation, wantedRotation, Time.deltaTime * rotationDamping);
          
            if(!isSwiping/*&& !targetSkateboard.GetController().Brake()&& Mathf.Abs( targetSkateboard.GetController().GetSteer())<Mathf.Epsilon*/)
            {
                if(swipeRecoveTime<followCameraConfig.SwipeRecoverTime)
                {
                    swipeRecoveTime += frameTime;
                }
                else
                {
                    swipeDelta    = Vector2.Lerp(swipeDelta, Vector2.zero, /*frameTime **/ followCameraConfig.SwipeRecoverRatio/100);
                }
            }
            //自由视角调整

            Vector3 camFreeRotateY = targetTransform.up;
            mTransform.RotateAround(targetTransform.position, camFreeRotateY, swipeDelta.x*followCameraConfig.HAngleSwipe);
            Vector3 camFreeRotateX = mTransform.right;
            mTransform.RotateAround(targetTransform.position, camFreeRotateX, -swipeDelta.y*followCameraConfig.VAngleSwipe);

            ProcessCameraCollision(lookAtPosition);
        }
        protected override void ResetCamera()
        {
            smoothTargetRotation = targetTransform.rotation;
        }

        Vector2 swipeDelta;
        bool isSwiping =false;
        float swipeRecoveTime = 0;
        protected virtual void OnSwipe(Gesture gesture)
        {
//             if (atRightScreen(gesture))
//                 return;
            if(!IsInPlane())
            {
                return;
            }
            Vector2 moveDelta = gesture.deltaPosition;
            moveDelta.x /= Screen.width;
            moveDelta.y /= Screen.height;
            swipeDelta += moveDelta;
        }
//         bool atRightScreen(Gesture gesture)
//         {
//             Vector2 refRes = UIManager.GetInstance().GetRefrenceResolution();
//             Vector2 pos = UIHelper.EasyTouchPosToUnityUIPos(gesture.position, ref refRes);
//             return (pos.x > 0);
//         }
        protected virtual void OnSwipeStart(Gesture gesture)
        {
            if (!IsInPlane())
            {
                return;
            }
            isSwiping = true;
        }
        protected virtual void OnSwipeEnd(Gesture gesture)
        {
            if (!IsInPlane())
            {
                return;
            }
            isSwiping = false;
            swipeRecoveTime = 0;
        }
        void OnStartFreeLook(IMessage msg)
        {
            isSwiping = true;
        }
        void OnFreeLooking(IMessage msg)
        {
            Vector2 moveDelta = (Vector2)msg.Data ;
            moveDelta.x /= Screen.width;
            moveDelta.y /= Screen.height;
            swipeDelta += moveDelta;
        }
        void OnEndFreeLook(IMessage msg)
        {
            isSwiping = false;
            swipeRecoveTime = 0;
        }
        void ProcessCameraCollision(Vector3 lookAtPosition)
        {
            Vector3 camPos = mTransform.position;
            Vector3 targetPos = lookAtPosition;
            RaycastHit hitInfo;
            bool hit = Physics.Raycast(targetPos, camPos - targetPos, out hitInfo, (camPos - targetPos).magnitude, ~LayerDefine.carLayerMask, QueryTriggerInteraction.Ignore);
            if(hit)
            {
                mTransform.position = hitInfo.point;
            }
        }
        protected override void Release()
        {
            base.Release();
            MessageDispatcher.RemoveListener(MessageDefine.StartFreeLook, OnStartFreeLook);
            MessageDispatcher.RemoveListener(MessageDefine.EndFreeLook, OnEndFreeLook);
            MessageDispatcher.RemoveListener(MessageDefine.FreeLooking, OnFreeLooking);
            //             HRacing.InputManager.GetInstance().UnregisterAction(HRacing.InputType.Swipe, OnSwipe);
            //             HRacing.InputManager.GetInstance().UnregisterAction(HRacing.InputType.SwipeStart, OnSwipeStart);
            //             HRacing.InputManager.GetInstance().UnregisterAction(HRacing.InputType.SwipeEnd, OnSwipeEnd);
        }
//         void OnFlightStatusChanged(IMessage msg)
//         {
//             int currStatus = (int)msg.Data;
//             gameObject.SetActive(currStatus == FlightStatusConst.Normal);
//         }
        bool IsInPlane()
        {
            if (GameManager.GetInstance().GetMyCar() != null)
                return GameManager.GetInstance().GetMyCar().GetPlayer().FlightStatus == FlightStatusConst.InPlane;
            else
                return false;
        }
    }

}