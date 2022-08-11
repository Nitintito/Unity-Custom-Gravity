using UnityEngine;
using System.Collections;

[RequireComponent(typeof(Camera))]
public class PlayerCamera : MonoBehaviour
{
	[SerializeField]
	Transform focus = default;

	[SerializeField, Range(1f, 20f)]
	float distance = 10f;

	[SerializeField, Min(0f)]
	float focusRadius = 2f;

	[SerializeField, Range(0f, 1f)]
	float focusCentering = 0.5f;

	[SerializeField, Range(1f, 360f)]
	float rotationSpeed = 90f;

	[SerializeField, Range(-89f, 89f)]
	float minVerticalAngle = -45f, maxVerticalAngle = 45f;

	[SerializeField, Min(0f)]
	float upAlignmentSpeed = 360f;

	[SerializeField]
	LayerMask obstructionMask = -1;

	Camera playerCamera;
	Vector3 focusPoint, previousFocusPoint;

	Vector2 orbitAngles = new Vector2(45f, 0f);
	Quaternion gravityAlignment = Quaternion.identity;
	Quaternion orbitRotation;
	
	Quaternion lookRotation;

	Vector3 lookDirection;
	Vector3 lookPosition; //normal position with weapon hollisterd.

	Vector3 rectOffset;
	Vector3 rectPosition;


	Vector3 castFrom; // normal camra pos for raycasting


	Vector3 castLine;


	float castDistance;


	Vector3 castDirection;


	Vector3 CameraHalfExtends
	{
		get
		{
			Vector3 halfExtends;
			halfExtends.y =
				playerCamera.nearClipPlane *
				Mathf.Tan(0.5f * Mathf.Deg2Rad * playerCamera.fieldOfView);
			halfExtends.x = halfExtends.y * playerCamera.aspect;
			halfExtends.z = 0f;
			return halfExtends;
		}
	}

	void OnValidate()
	{
		if (maxVerticalAngle < minVerticalAngle)
		{
			maxVerticalAngle = minVerticalAngle;
		}
	}

	void Awake()
	{
		playerCamera = GetComponent<Camera>();
		focusPoint = focus.position;
		transform.localRotation = orbitRotation = Quaternion.Euler(orbitAngles);
	}

	void LateUpdate()
	{
		UpdateFocusPoint();
		UpdateGravityAlignment();

		if (ManualRotation())
        {
			ConstrainAngles();
			orbitRotation = Quaternion.Euler(orbitAngles);
        }

		lookRotation = gravityAlignment * orbitRotation;

		lookDirection = lookRotation * Vector3.forward;
		lookPosition = focusPoint - lookDirection * distance; //normal position with weapon hollisterd.

		rectOffset = lookDirection * playerCamera.nearClipPlane;
		rectPosition = lookPosition + rectOffset;

		castFrom = focus.position; // normal camra pos for raycasting

		castLine = rectPosition - castFrom;

		castDistance = castLine.magnitude;

		castDirection = castLine / castDistance;

		    if (Physics.BoxCast(castFrom, CameraHalfExtends, castDirection, out RaycastHit hit,lookRotation, castDistance, obstructionMask))
		    {
			    rectPosition = castFrom + castDirection * hit.distance;
			    lookPosition = rectPosition - rectOffset;
		    }
	
		    transform.SetPositionAndRotation(lookPosition, lookRotation);
	}

	void UpdateFocusPoint()
	{
		float t = 1f;
		previousFocusPoint = focusPoint;
		Vector3 targetPoint = focus.position;

		if (focusRadius > 0f)
		{
			float distance = Vector3.Distance(targetPoint, focusPoint);
			if (distance > 0.01f && focusCentering > 0f)
			{
				t = Mathf.Pow(1f - focusCentering, Time.unscaledDeltaTime);
			}
			if (distance > focusRadius)
			{
				t = Mathf.Min(t, focusRadius / distance);
			}
			focusPoint = Vector3.Lerp(targetPoint, focusPoint, t);
			//transform.SetPositionAndRotation(lookPosition, lookRotation);
		}
		else
		{
			focusPoint = targetPoint;
		}
	}

	bool ManualRotation()
	{
		Vector2 input = new Vector2(
			Input.GetAxis("Vertical Camera"),
			Input.GetAxis("Horizontal Camera"));

		const float e = 0.001f;
		if (input.x < -e || input.x > e || input.y < -e || input.y > e)
		{
			orbitAngles += rotationSpeed * Time.unscaledDeltaTime * input;
			return true;
		}
		return false;
	}

	void ConstrainAngles() //constrain how far you can rotat the camra in the y direction.
	{
		orbitAngles.x = Mathf.Clamp(orbitAngles.x, minVerticalAngle, maxVerticalAngle);

		if (orbitAngles.y < 0f)
		{
			orbitAngles.y += 360f;
		}
		else if (orbitAngles.y >= 360f)
		{
			orbitAngles.y -= 360f;
		}
	}

	IEnumerator AnimateCameraToPos(Vector3 aimPos)
    {
        SetCameraToPos(aimPos);
        yield return new WaitForSeconds(0.5f);
    }
    
    void SetCameraToPos(Vector3 givenPos)
    {       
        //transform.position = givenTrans.position;
        transform.position = Vector3.Lerp(transform.position, givenPos, 3 * Time.deltaTime);
    }

	void UpdateGravityAlignment()
	{
		/*
        if (Inventory.weaponHollisterd)
        {
			gravityAlignment = Quaternion.FromToRotation(gravityAlignment * Vector3.up, CustomGravity.GetUpAxis(focusPoint)) * gravityAlignment;
        }
		else
			gravityAlignment = Quaternion.FromToRotation(gravityAlignment * Vector3.up, CustomGravity.GetUpAxis(focusPointGun)) * gravityAlignment;
		*/
		
		Vector3 fromUp = gravityAlignment * Vector3.up;
		Vector3 toUp = CustomGravity.GetUpAxis(focusPoint);
     
		float dot = Mathf.Clamp(Vector3.Dot(fromUp, toUp), -1f, 1f);
		float angle = Mathf.Acos(dot) * Mathf.Rad2Deg;
		float maxAngle = upAlignmentSpeed * Time.deltaTime;
		Quaternion newAlignment = Quaternion.FromToRotation(fromUp, toUp) * gravityAlignment;

		if (angle <= maxAngle)
		{
			gravityAlignment = newAlignment;
		}
		else
		{
			gravityAlignment = Quaternion.SlerpUnclamped(gravityAlignment, newAlignment, maxAngle / angle);
		}
	}
}
