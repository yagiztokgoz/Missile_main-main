using UnityEngine;

[RequireComponent(typeof(TrailRenderer))]
public class PlaybackTrail : MonoBehaviour
{
    private enum TrailColorMode
    {
        Kirmizi,
        Lacivert,
    }

    [SerializeField] private TrailColorMode colorMode = TrailColorMode.Kirmizi;
    [SerializeField] private Transform followSource;
    [SerializeField] private float startWidth = 0.6f;
    [SerializeField] private float endWidth = 0.05f;
    [SerializeField] private float trailTime = 2.5f;
    [SerializeField] private float minVertexDistance = 0.1f;

    private TrailRenderer trailRenderer;

    private void Awake()
    {
        trailRenderer = GetComponent<TrailRenderer>();
        Configure();
    }

    private void OnEnable()
    {
        if (trailRenderer == null)
        {
            trailRenderer = GetComponent<TrailRenderer>();
        }

        Configure();
        trailRenderer.Clear();
    }

    private void LateUpdate()
    {
        if (followSource == null)
        {
            return;
        }

        transform.position = followSource.position;
        transform.rotation = followSource.rotation;
    }

    public void ClearTrail()
    {
        if (trailRenderer != null)
        {
            trailRenderer.Clear();
        }
    }

    private void Configure()
    {
        if (trailRenderer == null)
        {
            return;
        }

        trailRenderer.time = trailTime;
        trailRenderer.startWidth = startWidth;
        trailRenderer.endWidth = endWidth;
        trailRenderer.minVertexDistance = minVertexDistance;
        trailRenderer.alignment = LineAlignment.View;
        trailRenderer.emitting = true;
        trailRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        trailRenderer.receiveShadows = false;

        Color startColor;
        Color endColor;
        GetColors(out startColor, out endColor);

        Gradient gradient = new Gradient();
        gradient.SetKeys(
            new GradientColorKey[]
            {
                new GradientColorKey(startColor, 0f),
                new GradientColorKey(endColor, 1f),
            },
            new GradientAlphaKey[]
            {
                new GradientAlphaKey(startColor.a, 0f),
                new GradientAlphaKey(endColor.a, 1f),
            }
        );
        trailRenderer.colorGradient = gradient;

        if (trailRenderer.sharedMaterial == null)
        {
            Shader shader = Shader.Find("Sprites/Default");
            if (shader != null)
            {
                trailRenderer.sharedMaterial = new Material(shader);
            }
        }
    }

    private void GetColors(out Color startColor, out Color endColor)
    {
        switch (colorMode)
        {
            case TrailColorMode.Lacivert:
                startColor = new Color(0.08f, 0.16f, 0.45f, 1f);
                endColor = new Color(0.08f, 0.16f, 0.45f, 0f);
                break;

            default:
                startColor = new Color(0.85f, 0.12f, 0.12f, 1f);
                endColor = new Color(0.85f, 0.12f, 0.12f, 0f);
                break;
        }
    }
}
