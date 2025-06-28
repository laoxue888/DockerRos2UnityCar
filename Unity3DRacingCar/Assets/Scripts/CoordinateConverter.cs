using UnityEngine;

public class CoordinateConverter : MonoBehaviour
{
    public Transform referenceObject; // 参考坐标系物体
    public Transform targetObject;   // 要转换的物体

    void Update()
    {
        if (referenceObject != null && targetObject != null)
        {
            // 计算局部位置和旋转
            Vector3 localPos = referenceObject.InverseTransformPoint(targetObject.position);
            Quaternion localRot = Quaternion.Inverse(referenceObject.rotation) * targetObject.rotation;

            Debug.Log("局部位置: " + localPos);
            Debug.Log("局部旋转: " + localRot.eulerAngles);

            // 可视化调试
            Debug.DrawLine(referenceObject.position, targetObject.position, Color.red);
            Debug.DrawRay(targetObject.position, targetObject.forward * 2, Color.blue);
        }
    }
}