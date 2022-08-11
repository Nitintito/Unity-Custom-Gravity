using UnityEngine;



[RequireComponent (typeof(Rigidbody))]

public class CustomGravityRigidBody : MonoBehaviour
{
    Rigidbody rb;
    float floatDelay;
    [SerializeField]
    bool floatToSleep = false;
    [SerializeField]
    bool deSpawn;
    [SerializeField]
    float deSpawnTimer = 10;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = false;
    }

    private void Update()
    {
        if (deSpawn && rb.isKinematic == false)
        {
            Destroy(gameObject, deSpawnTimer);
        }
    }

    void FixedUpdate()
    {
        if (floatToSleep)
        {
            if (rb.IsSleeping())
            {
                floatDelay = 0f;
                return;
            }
            if (rb.velocity.sqrMagnitude < 0.001f)
            {
                floatDelay += Time.deltaTime;
                if (floatDelay >=1f)
                {
                return;
                }
            }
            else
            {
                floatDelay = 0f;
            }
        }

        rb.AddForce(CustomGravity.GetGravity(rb.position), ForceMode.Acceleration); 
    }
}

