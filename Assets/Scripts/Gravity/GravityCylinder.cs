using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// TODO add range to top and bot

public class GravityCylinder : GravitySource
{
    [SerializeField]
    float gravity = 9.81f; //adjust the gravitational force

    [SerializeField, Min(0f)]
    float outerRadius = 10f, outerFalloffRadius = 15f; //adjust the radius of the gravitatinal pull
    [SerializeField, Min(0f)]
    float innerRadius = 5f, innerFalloffRadius = 1f; //adjust the radius of the gravitational puhs (can be used for inverted spheres when you walk inside it)

    float innerFalloffFactor, outerFalloffFactor;

    void Awake()
    {
        OnValidate();
    }

    public override Vector3 GetGravity(Vector3 position) //calculates the gravitational pull force.
    {
        //Debug.Log(position.y - transform.localPosition.y); // yeessss???

        Vector3 vector;
        if(position.y - transform.localPosition.y > transform.localScale.y) // if on top of cylinder
        {
            Debug.Log("On Top");
            vector = transform.position - position; // todo add corect gravity
        }
        else if (position.y - transform.localPosition.y < -transform.localScale.y) // i f on bottom
        {
            Debug.Log("On Bot");
            vector = transform.position - position; // todo add corect gravity
        }
        else
        {
            Debug.Log("On Cylinder");
            vector = Vector3.ProjectOnPlane(transform.position - position, transform.up);
        }

        float distance = vector.magnitude;
        if (distance > outerFalloffRadius || distance < innerFalloffRadius)
        {
            return Vector3.zero;
        }

        float g = gravity / distance;
        if (distance > outerRadius) //creates gravitational pull towards the sphere
        {
            g *= 1f - (distance - outerRadius) * outerFalloffFactor;
        }
        else if (distance < innerRadius) // creats gravitational pull directed outwards from the spere (used for inverted spehres when you can walk inside)
        {
            g *= 1f - (innerRadius - distance) * innerFalloffFactor;
        }
        return g * vector;
    }

    void OnDrawGizmos()
    {
        Vector3 p = transform.position;
        //inner radius (for inverted spheres)
        if (innerFalloffRadius > 0f && innerFalloffRadius < innerRadius) //used for inverted spheres
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(p, innerFalloffRadius);
        }
        Gizmos.color = Color.yellow;
        if (innerRadius > 0f && innerRadius < outerRadius) //used for inverted spheres
        {
            Gizmos.DrawWireSphere(p, innerRadius);
        }
        //Outer radius
        Gizmos.DrawWireSphere(p, outerRadius); //Yellow draws the sphere where gravity is at 100% force.
        if (outerFalloffRadius > outerRadius) //Cyan draws the sphere where gravity starts to fall off
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(p, outerFalloffRadius);
        }
    }

    private void OnValidate()
    {
        innerFalloffRadius = Mathf.Max(innerFalloffRadius, 0f);
        innerRadius = Mathf.Max(innerRadius, innerFalloffRadius);
        outerRadius = Mathf.Max(outerRadius, innerRadius);
        outerFalloffRadius = Mathf.Max(outerRadius, outerFalloffRadius);

        innerFalloffFactor = 1f / (innerRadius - innerFalloffRadius);
        outerFalloffFactor = 1f / (outerFalloffRadius - outerRadius);
    }
}
