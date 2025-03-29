using System;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.Tf2;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class ROS2TalkerDemo : MonoBehaviour
{
    const string k_DemoTalkerTopic = "/demo_talker";
    const string k_ImageTopic = "/image_talker";

    [SerializeField]
    double m_PublishRateHz = 20f;
    double PublishPeriodSeconds => 1.0f / m_PublishRateHz;
    double m_LastPublishTimeSeconds;
    bool ShouldPublishMessage => Clock.NowTimeInSeconds > m_LastPublishTimeSeconds + PublishPeriodSeconds;

    ROSConnection m_ROS;
    Camera m_Camera;

    private int i;

    // Start is called before the first frame update
    void Start()
    {
        m_ROS = ROSConnection.GetOrCreateInstance();
        if (m_ROS == null)
        {
            Debug.LogError("ROSConnection instance is null!");
        }
        else
        {
            m_ROS.RegisterPublisher<StringMsg>(k_DemoTalkerTopic);
            m_ROS.RegisterPublisher<ImageMsg>(k_ImageTopic);
        }

        m_LastPublishTimeSeconds = Clock.time + PublishPeriodSeconds;

        GameObject cameraObject = GameObject.Find("Camera"); // 根据名称查找相机
        if (cameraObject != null)
        {
            m_Camera = cameraObject.GetComponent<Camera>();
            if (m_Camera == null)
            {
                Debug.LogError("Camera component is not found on the GameObject!");
            }
        }
        else
        {
            Debug.LogError("Main Camera GameObject is not found!");
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (ShouldPublishMessage)
        {
            if (m_ROS == null || m_Camera == null)
            {
                Debug.LogError("m_ROS or m_Camera is null!");
                return;
            }

            i++;
            if (i > 1000000)
            {
                i = 0;
            }
            var msgDemoTalker = new StringMsg
            {
                data = "Hello, ROS2! send: " + i
            };
            m_ROS.Publish(k_DemoTalkerTopic, msgDemoTalker);

            // 获取相机图像
            Texture2D texture = new Texture2D(m_Camera.pixelWidth, m_Camera.pixelHeight, TextureFormat.RGB24, false);
            RenderTexture renderTexture = new RenderTexture(m_Camera.pixelWidth, m_Camera.pixelHeight, 24);
            m_Camera.targetTexture = renderTexture;
            m_Camera.Render();
            RenderTexture.active = renderTexture;
            texture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
            texture.Apply();
            m_Camera.targetTexture = null;
            RenderTexture.active = null;
            Destroy(renderTexture);

            // 倒转图像
            Color[] pixels = texture.GetPixels();
            Color[] flippedPixels = new Color[pixels.Length];
            int width = texture.width;
            int height = texture.height;

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    flippedPixels[(height - 1 - y) * width + x] = pixels[y * width + x];
                }
            }

            texture.SetPixels(flippedPixels);
            texture.Apply();

            byte[] imageData = texture.GetRawTextureData();

            var msgImage = new ImageMsg
            {
                height = (uint)texture.height,
                width = (uint)texture.width,
                encoding = "rgb8",
                is_bigendian = 0,
                step = (uint)(texture.width * 3),
                data = imageData
            };

            m_ROS.Publish(k_ImageTopic, msgImage);

            Debug.Log("Published message: " + msgDemoTalker.data);
            m_LastPublishTimeSeconds = Clock.FrameStartTimeInSeconds;
        }
    }
}

