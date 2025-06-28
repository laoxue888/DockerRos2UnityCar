using UnityEngine;

public class CoordinateConverter : MonoBehaviour
{
    public Transform referenceObject; // �ο�����ϵ����
    public Transform targetObject;   // Ҫת��������

    void Update()
    {
        if (referenceObject != null && targetObject != null)
        {
            // ����ֲ�λ�ú���ת
            Vector3 localPos = referenceObject.InverseTransformPoint(targetObject.position);
            Quaternion localRot = Quaternion.Inverse(referenceObject.rotation) * targetObject.rotation;

            Debug.Log("�ֲ�λ��: " + localPos);
            Debug.Log("�ֲ���ת: " + localRot.eulerAngles);

            // ���ӻ�����
            Debug.DrawLine(referenceObject.position, targetObject.position, Color.red);
            Debug.DrawRay(targetObject.position, targetObject.forward * 2, Color.blue);
        }
    }
}