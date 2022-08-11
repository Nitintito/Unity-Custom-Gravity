using UnityEngine;



public class GravityPlane : GravitySource
{

    /*  This script add sources of gravity in 1 given direction depending on the rotation of the object
     *  Put this script on all PLANES that are supose to affect gravity in 1 shosen directinon.
     */
    [SerializeField]
    float gravity = 9.81f; //Gravitational pull force and distance.
    [SerializeField, Min(0f)]
    float range = 1f; //The range of the gravitational pull.
    [SerializeField] bool useFallOffDistance = false; // makes the gravity Weeker the further away a objekt is
    [SerializeField] float gizmoScaleX = 1f, gizmoScaleZ =1f;

    public override Vector3 GetGravity (Vector3 position)//Returns gravity force and direction.
    {
        Vector3 up = transform.up; //Up is allways the same as the objects up direction(transform.up). If the item is rotated the up direction is rotated to.
        float distance = Vector3.Dot(up, position - transform.position); //use to determen the range of gravitational pull.
        if (distance > range) //if distance is larger then range = no gravitational pull.
        {
            return Vector3.zero;
        }
        float g = -gravity;
        if (distance > 0f && useFallOffDistance) //makes the gravitational pull weeker the longer you are from the plane.
        {
            g *= 1f - distance / range;
        }
        else
        {
            g = -gravity; // if not using falloffdistance use the same gravity 
        }
        return g * up; // returns the gravity direction.
    }

    void OnDrawGizmos()//Draws lines so we know where the gravity feelds are
    {
        Vector3 scale = transform.localScale;
        scale.y = range;
        Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, scale);
        Vector3 size = new Vector3(gizmoScaleX, 0, gizmoScaleZ);
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(Vector3.zero, size); //yelow visualizing the plane size.
        if (range > 0)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireCube(Vector3.up, size); //cyan visualising the gravitational pull range.
        }
    }
}

