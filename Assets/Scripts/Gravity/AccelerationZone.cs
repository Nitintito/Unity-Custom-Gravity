using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AccelerationZone : MonoBehaviour
{
    [SerializeField]
    float acceleration = 10f, speed =10f;

    private void OnTriggerEnter(Collider other)
    {
        Rigidbody rb = other.attachedRigidbody;
        if (rb)
        {

            Accelerate(rb);
        }
    }

    private void OnTriggerStay(Collider other)
    {
        Rigidbody rb = other.attachedRigidbody;
        if (other)
        {
            Accelerate(rb);
        }
    }

    void Accelerate(Rigidbody rb)
    {
        Vector3 velocity = transform.InverseTransformDirection(rb.velocity);
        if (velocity.y >= speed)
        {
            return;
        }

        if (acceleration > 0f)
        {
            velocity.y = Mathf.MoveTowards(velocity.y, speed, acceleration * Time.deltaTime);
        }

        else
        {
            velocity.y = speed;
        }

        rb.velocity = transform.TransformDirection(velocity);

        if (rb.TryGetComponent(out PlayerMovement player)) // prevents snap to gound so the player can be affected by the zones
        {
            player.PreventSnapToGround(); // prevent snap to gound
        }

    }

}

