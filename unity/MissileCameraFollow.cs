using UnityEngine;

public class MissileCameraFollow : MonoBehaviour
{
    [SerializeField] private Transform target;
    [SerializeField] private Vector3 offset = new Vector3(0f, 6f, -18f);
    [SerializeField] private float positionSmoothTime = 0.05f;
    [SerializeField] private float rotationLerpSpeed = 18f;
    [SerializeField] private bool alignWithTargetForward = true;
    [SerializeField] private bool snapPosition = false;
    [SerializeField] private bool snapRotation = true;
    [SerializeField] private Vector3 lookOffset = new Vector3(0f, 0f, 0f);

    private Vector3 velocity;
    private bool hasSnappedToTarget;

    private void OnEnable()
    {
        hasSnappedToTarget = false;
        velocity = Vector3.zero;
    }

    private void LateUpdate()
    {
        if (target == null)
        {
            return;
        }

        Vector3 desiredOffset = alignWithTargetForward
            ? target.TransformDirection(offset)
            : offset;
        Vector3 desiredPosition = target.position + desiredOffset;
        Vector3 lookTarget = target.position + target.TransformDirection(lookOffset);

        if (!hasSnappedToTarget)
        {
            transform.position = desiredPosition;
            Vector3 initialLook = lookTarget - transform.position;
            if (initialLook.sqrMagnitude > 1e-6f)
            {
                transform.rotation = Quaternion.LookRotation(initialLook, Vector3.up);
            }
            hasSnappedToTarget = true;
            return;
        }

        if (snapPosition)
        {
            transform.position = desiredPosition;
            velocity = Vector3.zero;
        }
        else
        {
            transform.position = Vector3.SmoothDamp(
                transform.position,
                desiredPosition,
                ref velocity,
                Mathf.Max(0.001f, positionSmoothTime)
            );
        }

        Vector3 lookDirection = lookTarget - transform.position;
        if (lookDirection.sqrMagnitude < 1e-6f)
        {
            return;
        }

        Quaternion desiredRotation = Quaternion.LookRotation(lookDirection, Vector3.up);
        if (snapRotation)
        {
            transform.rotation = desiredRotation;
        }
        else
        {
            transform.rotation = Quaternion.Slerp(
                transform.rotation,
                desiredRotation,
                rotationLerpSpeed * Time.deltaTime
            );
        }
    }
}
