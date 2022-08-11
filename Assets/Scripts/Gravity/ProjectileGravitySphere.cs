using UnityEngine;
using System.Collections.Generic;

public class ProjectileGravitySphere : MonoBehaviour
{
    [SerializeField]
    float gravity = 100f; //adjust the gravitational force
    [SerializeField]
    bool stopAddingGravityOnExit;
    [SerializeField]
    bool multipleProjectileSupport = true;
    bool insideGravityField = false;
    
    Collider curretnProjectile;
    List<GameObject> projectileList = new List<GameObject>();

    private void FixedUpdate()
    {
        if (multipleProjectileSupport)
        {
            projectileList.RemoveAll(projectile => projectile == null);
            foreach (GameObject projectile in projectileList)
            {
                if (projectileList.Contains(projectile))
                {
                    projectile.GetComponent<Rigidbody>().AddForce(gravity * Vector3.Normalize(transform.position - projectile.transform.position), ForceMode.Acceleration);
                }
            }
        }
        else
        {
            if (curretnProjectile != null && insideGravityField)
            {
                curretnProjectile.GetComponent<Rigidbody>().AddForce(gravity * Vector3.Normalize(transform.position - curretnProjectile.transform.position), ForceMode.Acceleration);
            }
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Projectile") && !projectileList.Contains(other.gameObject))
        {
            // curretnProjectile = other;
            projectileList.Add(other.gameObject);
        }

        if (other.CompareTag("Projectile"))
        {
            curretnProjectile = other;
            insideGravityField = true;
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (multipleProjectileSupport && stopAddingGravityOnExit)
        {
            if (projectileList.Contains(other.gameObject))
            {
                projectileList.Remove(other.gameObject);
            }
        }
        else
        {
            if (other == curretnProjectile && stopAddingGravityOnExit)
            {
                insideGravityField = false;
            }
        }
    }
}

