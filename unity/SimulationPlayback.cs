using UnityEngine;

public class SimulationPlayback : MonoBehaviour
{
    [SerializeField] private TextAsset jsonFile;
    [SerializeField] private Transform missileTransform;
    [SerializeField] private Transform targetTransform;
    [SerializeField] private PlaybackTrail missileTrail;
    [SerializeField] private PlaybackTrail targetTrail;
    [SerializeField] private float playbackSpeed = 1f;
    [SerializeField] private bool loop = true;
    [SerializeField] private bool playOnStart = true;

    private PlaybackData data;
    private float playbackTime;
    private int frameIndex;
    private bool isPlaying;

    [System.Serializable]
    private class PlaybackData
    {
        public PlaybackMetadata metadata;
        public PlaybackFrame[] frames;
    }

    [System.Serializable]
    private class PlaybackMetadata
    {
        public float duration_s;
    }

    [System.Serializable]
    private class PlaybackFrame
    {
        public float time;
        public PlaybackPose missile;
        public PlaybackPose target;
    }

    [System.Serializable]
    private class PlaybackPose
    {
        public PlaybackVec3 position;
        public PlaybackVec3 forward;
        public PlaybackVec3 up;
    }

    [System.Serializable]
    private class PlaybackVec3
    {
        public float x;
        public float y;
        public float z;

        public Vector3 ToVector3()
        {
            return new Vector3(x, y, z);
        }
    }

    private void Awake()
    {
        if (jsonFile == null)
        {
            Debug.LogWarning("SimulationPlayback: jsonFile atanmamis.");
            return;
        }

        data = JsonUtility.FromJson<PlaybackData>(jsonFile.text);
        if (data == null || data.frames == null || data.frames.Length == 0)
        {
            Debug.LogWarning("SimulationPlayback: JSON bos veya gecersiz.");
            return;
        }

        playbackTime = 0f;
        frameIndex = 0;
        isPlaying = playOnStart;
        ClearTrails();
        ApplyFrame(data.frames[0]);
    }

    private void Update()
    {
        if (!isPlaying || data == null || data.frames == null || data.frames.Length == 0)
        {
            return;
        }

        playbackTime += Time.deltaTime * playbackSpeed;
        float duration = Mathf.Max(data.metadata.duration_s, data.frames[data.frames.Length - 1].time);

        if (loop && duration > 0f)
        {
            while (playbackTime > duration)
            {
                playbackTime -= duration;
                frameIndex = 0;
            }
        }
        else
        {
            playbackTime = Mathf.Min(playbackTime, duration);
        }

        while (frameIndex < data.frames.Length - 2 && data.frames[frameIndex + 1].time < playbackTime)
        {
            frameIndex++;
        }

        PlaybackFrame a = data.frames[frameIndex];
        PlaybackFrame b = data.frames[Mathf.Min(frameIndex + 1, data.frames.Length - 1)];
        float segment = Mathf.Max(b.time - a.time, 1e-5f);
        float t = Mathf.Clamp01((playbackTime - a.time) / segment);

        ApplyInterpolatedPose(missileTransform, a.missile, b.missile, t);
        ApplyInterpolatedPose(targetTransform, a.target, b.target, t);
    }

    public void Play()
    {
        isPlaying = true;
    }

    public void Pause()
    {
        isPlaying = false;
    }

    public void Restart()
    {
        playbackTime = 0f;
        frameIndex = 0;
        ClearTrails();
        if (data != null && data.frames != null && data.frames.Length > 0)
        {
            ApplyFrame(data.frames[0]);
        }
    }

    private void ClearTrails()
    {
        if (missileTrail != null)
        {
            missileTrail.ClearTrail();
        }

        if (targetTrail != null)
        {
            targetTrail.ClearTrail();
        }
    }

    private void ApplyFrame(PlaybackFrame frame)
    {
        ApplyPose(missileTransform, frame.missile);
        ApplyPose(targetTransform, frame.target);
    }

    private static void ApplyInterpolatedPose(Transform tr, PlaybackPose a, PlaybackPose b, float t)
    {
        if (tr == null || a == null || b == null)
        {
            return;
        }

        Vector3 pos = Vector3.Lerp(a.position.ToVector3(), b.position.ToVector3(), t);
        Quaternion rotA = Quaternion.LookRotation(a.forward.ToVector3(), a.up.ToVector3());
        Quaternion rotB = Quaternion.LookRotation(b.forward.ToVector3(), b.up.ToVector3());

        tr.position = pos;
        tr.rotation = Quaternion.Slerp(rotA, rotB, t);
    }

    private static void ApplyPose(Transform tr, PlaybackPose pose)
    {
        if (tr == null || pose == null)
        {
            return;
        }

        tr.position = pose.position.ToVector3();
        tr.rotation = Quaternion.LookRotation(pose.forward.ToVector3(), pose.up.ToVector3());
    }
}
