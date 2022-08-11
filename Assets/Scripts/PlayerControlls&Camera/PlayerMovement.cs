using UnityEngine;
/*
 * Made by Jason Sandelin Francis
 */
[RequireComponent(typeof(Rigidbody))]
public class PlayerMovement : MonoBehaviour
{
    [SerializeField]
    Transform playerInputSpace = default; // gör så att rörelser anpassar sig efter kamera direction.
    [SerializeField, Range(0f, 100f)]
    float maxSpeed = 20, maxClimbSpeed = 5f;           // Hastighet på spelaren
    [SerializeField, Range(0f, 360)]
    float rotationSpeed = 20;
    [SerializeField, Range(0f, 100f)]
    float maxAcceleration = 50f, maxAirAcceleration = 20f, maxClimbAcceleration = 40f; // axelerationshastighet
    [SerializeField, Range(0f, 10f)]
    float jumpHeight = 2f;          //Hopp höjd.
    [SerializeField, Range(0, 5)]
    int maxAirJumps = 0;            // om man ska kunna dubble jumpa
    [SerializeField]
    public int dashForce = 22;            // om man ska kunna dubble jumpa
    [SerializeField, Range(0, 90)]
    float maxGroundAngle = 45f, maxStairsAngle = 50f; //maximal vinkel mark och trappor
    [SerializeField, Range(90, 180)]
    float maxClimbAngle = 140f;     // Max climb angle
    [SerializeField, Range(0f, 100f)]
    float maxSnapSpeed = 100f;      //hur fort man kan röra sig innan SnapToGround ignorerar (så man fltyger över kullar man e för snabb)
    [SerializeField, Min(0f)]
    float groundProbeDistance = 2.5f, climbProbeDistance = 2.5f;     // Hur långt raycasting ska kolla för att avgöra om den ska SnapToGround (gör så man inte flyger över kullar när man når toppen)
    [SerializeField]
    LayerMask probeMask = -1, stairsMask = -1, climbMask = -1; // används för raycasting och för att avgöra om spelaren klättrar eller är på trappor.
    Rigidbody rb, connectedRb, previousConnectedRb; // anbvänds för att kolla hur olika colliders förhåller sig till varandra
    Vector2 playerInput;
    Vector3 movementDirection; //check player movedirection. Used to rotate charecter towards move direction
    Vector3 velocity, connectionVelocity; // för hastighet på spelaren
    Vector3 contactNormal, steepNormal, climbNormal, lastClimbNormal; // kollar normal vektorn av kontaktobjektet
    Vector3 connectionWorldPosition, connectionLocalPosition; //
    //gravity
    Vector3 upAxis, rightAxis, forwardAxis; //används till att avgöra directions oavsett vilket håll gravitation drar en mot.
    Quaternion gravityAlignment = Quaternion.identity; //for gravity alignment.
    Vector3 focusPoint; //for gravity alignment.
    [SerializeField]
    float upAlignmentSpeed = 360f; // how fast will the player adjust to changes in gravity.
    bool desiredJump, desiresLaunchCannon; // inputs för om spelaren vill hoppa eller klättra
    int groundContactCount, steepContactCount, climbContactCount; // används för att kolla hur många kontaktpunkter spelaren har för att kunna avgöra normalvektorn av de punkterna.
    bool OnGround => groundContactCount > 0; // kollar om man rör flera vinklar av mark och normalizerar dem för att avgöra hur man hoppar.

    bool Climbing => climbContactCount > 0 && stepsSinceLastJump > 2; // kollar physic stegen sen senaste jump mm. Detta för att stänga av snapToGround
    int jumpPhase; // kollar om man hoppat	TROR JAG SKA KOLLA IGENOM //	UPPDATE DESCRIPTION!!!!
    float minGroundDotProduct, minStairsDotProduct, minClimbDotProduct; //kollar dotproduct på olika kontaktytor
    int stepsSinceLastGrounded, stepsSinceLastJump; // kollar hur många physic steg som gått sen olika actions

    //Launch Cannon
    private bool insideLaunchCannon;
    private bool insideLaunchCannonCart;

    Animator animator;

    void OnValidate() //i awake så sätts de maximala lutningen på olika objekt för att avgöra om man kan klättra eller gå osv.
    {
        minGroundDotProduct = Mathf.Cos(maxGroundAngle * Mathf.Deg2Rad);
        minStairsDotProduct = Mathf.Cos(maxStairsAngle * Mathf.Deg2Rad);
        minClimbDotProduct = Mathf.Cos(maxClimbAngle * Mathf.Deg2Rad);
    }

    void Awake()
    {
        Cursor.lockState = CursorLockMode.Locked;
        rb = GetComponent<Rigidbody>();
        rb.useGravity = false; // VIKTIGT. Stänger av gravity på RB för att använda custim gravity som jag skapat.
        OnValidate();
        animator = GetComponentInChildren<Animator>();
    }

    void Update()
    {
        //movement inputs
        playerInput.x = Input.GetAxis("Horizontal");
        playerInput.y = Input.GetAxis("Vertical");
        playerInput = Vector2.ClampMagnitude(playerInput, 1f);

        PlayerRotation();

        if (playerInputSpace) // player input space e kameran som gör att player.forward är lika med camera.forward
        {
            rightAxis = ProjectDirectionOnPlane(playerInputSpace.right, upAxis);
            forwardAxis = ProjectDirectionOnPlane(playerInputSpace.forward, upAxis);
        }
        else // om man inte har någon kamera tillagd som inputspace
        {
            rightAxis = ProjectDirectionOnPlane(Vector3.right, upAxis);
            forwardAxis = ProjectDirectionOnPlane(Vector3.forward, upAxis);
        }

        desiredJump |= Input.GetButtonDown("Jump"); // om spelaren trycker "Jump" så aktiveras desiredjump boolen 
    }

    void FixedUpdate()
    {
        Vector3 gravity;
        if (upAxis.x != 0 || upAxis.y != 0 || upAxis.z != 0)
        {
            gravity = CustomGravity.GetGravity(rb.position, out upAxis);
            //Debug.Log(upAxis);
        }
        else
        {
            gravity = CustomGravity.GetGravity(rb.position, out upAxis);

           // Debug.Log("UpAxis = 0");
           // gravity = new Vector3(0, -20, 0);
           // upAxis = new Vector3(0, 1, 0);
        }


        UpdateState();
        AdjustVelocity();    

        if (desiredJump)
        {
            desiredJump = false;

            Jump(gravity);
        }
        else if (OnGround && velocity.sqrMagnitude < 0.01f)
        {

            velocity += contactNormal * (Vector3.Dot(gravity, contactNormal) * Time.deltaTime);
        }
        else
        {
            velocity += gravity * Time.deltaTime;
        }
        if (!insideLaunchCannonCart && !insideLaunchCannon)
            rb.velocity = velocity;
        else
            rb.velocity = Vector3.zero;
        ClearState();
    }

    void PlayerRotation()
    {
       
        Vector3 gravity = CustomGravity.GetGravity(rb.position, out upAxis);
       
        if (playerInput.x != 0 || playerInput.y != 0) // stops rotating when there is no inputs to avoid spazzing out. And rotates towards movment direction
        {
            //rotation towards movement direction.
            forwardAxis = transform.GetComponent<Rigidbody>().velocity; // gets the forwardAxis based on rb velocity
            transform.rotation = Quaternion.Lerp(transform.rotation, Quaternion.LookRotation(forwardAxis, transform.up), rotationSpeed * Time.deltaTime); //transfrom.rotate in the direction of forwardAxis
            //Graviry Alignment
            transform.rotation = Quaternion.LookRotation(gravity, transform.forward);
            transform.Rotate(Vector3.left, 90f);
        }
        else // if no momvement input
        {
            transform.rotation = Quaternion.LookRotation(gravity, transform.forward);
            transform.Rotate(Vector3.left, 90f);
        }
    }

    void ClearState() // används i fixed update för att återställa vissa saker.
    {
        groundContactCount = steepContactCount = climbContactCount = 0;
        contactNormal = steepNormal = climbNormal = Vector3.zero;
        connectionVelocity = Vector3.zero;
        previousConnectedRb = connectedRb;
        connectedRb = null;
    }

    void UpdateState() //Track how many physics steps there have been since we considered ourselves grounded.
    {
        stepsSinceLastGrounded += 1;
        stepsSinceLastJump += 1;
        velocity = rb.velocity;

        if (OnGround || SnapToGround() || CheckSteepContacts())
        {
            stepsSinceLastGrounded = 0;
            if (stepsSinceLastJump > 1)
            {
                jumpPhase = 0;
            }
            if (groundContactCount > 1)
            {
                contactNormal.Normalize();
            }
        }
        else
        {
            contactNormal = upAxis;
        }

        if (connectedRb)
        {
            if (connectedRb.isKinematic || connectedRb.mass >= rb.mass)
            {
                UpdateConnectionState();
            }
        }
    }

    void UpdateConnectionState()
    {
        if (connectedRb == previousConnectedRb)
        {
            Vector3 connectionMovement = connectedRb.transform.TransformPoint(connectionLocalPosition) - connectionWorldPosition;

            connectionVelocity = connectionMovement / Time.deltaTime;
        }
        connectionWorldPosition = rb.position;
        connectionLocalPosition = connectedRb.transform.InverseTransformPoint(connectionWorldPosition);
    }

    bool SnapToGround()
    {
        if (stepsSinceLastGrounded > 1 || stepsSinceLastJump <= 2)
        {
            return false;
        }

        float speed = velocity.magnitude;
        if (speed > maxSnapSpeed)
        {
            return false;
        }

        if (!Physics.Raycast(rb.position, -upAxis, out RaycastHit hit, groundProbeDistance, probeMask))
        {
            return false;
        }

        float upDot = Vector3.Dot(upAxis, hit.normal);
        if (upDot < GetMinDot(hit.collider.gameObject.layer))
        {
            return false;
        }

        groundContactCount = 1;
        contactNormal = hit.normal;
        float dot = Vector3.Dot(velocity, hit.normal);
        if (dot > 0f)
        {
            velocity = (velocity - hit.normal * dot).normalized * speed;
        }

        connectedRb = hit.rigidbody;
        return true;
    }

    public void PreventSnapToGround()
    {
        stepsSinceLastJump = -1;
    }

    bool CheckSteepContacts()
    {
        if (steepContactCount > 1)
        {
            steepNormal.Normalize();

            float upDot = Vector3.Dot(upAxis, steepNormal);

            if (upDot >= minGroundDotProduct)
            {
                steepContactCount = 0;
                groundContactCount = 1;
                contactNormal = steepNormal;
                return true;
            }
        }
        return false;
    }

    void AdjustVelocity()
    {
            float acceleration, speed;
            Vector3 xAxis, zAxis;

            acceleration = OnGround ? maxAcceleration : maxAirAcceleration;
            speed = maxSpeed;
            xAxis = rightAxis;
            zAxis = forwardAxis;
            
            xAxis = ProjectDirectionOnPlane(xAxis, contactNormal);
            zAxis = ProjectDirectionOnPlane(zAxis, contactNormal);
            Vector3 relativeVelocity = velocity - connectionVelocity;
            float currentX = Vector3.Dot(relativeVelocity, xAxis);
            float currentZ = Vector3.Dot(relativeVelocity, zAxis);
            float maxSpeedChange = acceleration * Time.deltaTime;
            float newX = Mathf.MoveTowards(currentX, playerInput.x * speed, maxSpeedChange);
            float newZ = Mathf.MoveTowards(currentZ, playerInput.y * speed, maxSpeedChange);
            velocity += xAxis * (newX - currentX) + zAxis * (newZ - currentZ);   
    }

    void Jump(Vector3 gravity)
    {
        Vector3 jumpDirection;
        if (OnGround)
        {
            jumpDirection = contactNormal;
        }
        else if (maxAirJumps > 0 && jumpPhase <= maxAirJumps)
        {
            if (jumpPhase == 0)
            {
                jumpPhase = 1;
            }
            jumpDirection = contactNormal;
        }
        else
        {
            return;
        }
        stepsSinceLastJump = 0;
        jumpPhase += 1;
        float jumpSpeed = Mathf.Sqrt(2f * gravity.magnitude * jumpHeight);
        jumpDirection = (jumpDirection + upAxis).normalized;
        float alignedSpeed = Vector3.Dot(velocity, jumpDirection);
        if (alignedSpeed > 0f)
        {
            jumpSpeed = Mathf.Max(jumpSpeed - alignedSpeed, 0f);
        }
        velocity += jumpDirection * jumpSpeed;
    }

    void OnCollisionEnter(Collision collision)
    {
        EvaluateCollision(collision);
    }

    void OnCollisionStay(Collision collision)
    {
        EvaluateCollision(collision);
    }

    void EvaluateCollision(Collision collision) // används för att kunna vara på rörliga objekt utan att ramla av.
    {
        int layer = collision.gameObject.layer; //kollar vad kollitionsobjectet har för layer
        float minDot = GetMinDot(layer); // ger mind dot på kollitionsobjektet
        for (int i = 0; i < collision.contactCount; i++)
        {
            Vector3 normal = collision.GetContact(i).normal;
            float upDot = Vector3.Dot(upAxis, normal);
            if (upDot >= minDot)
            {
                groundContactCount += 1;
                contactNormal += normal;
                connectedRb = collision.rigidbody;
            }
            else
            {
                if (upDot > -0.01f)
                {
                    steepContactCount += 1;
                    steepNormal += normal;
                    if (groundContactCount == 0)
                    {
                        connectedRb = collision.rigidbody;
                    }
                }
            }
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        EvaluateTriggerEnter(other);
    }

    private void OnTriggerStay(Collider other)
    {
        EvaluateTriggerEnter(other);
    }

    private void OnTriggerExit(Collider other)
    {
        EvaluateTriggerExit(other);
    }

    void EvaluateTriggerEnter(Collider other)
    {

    }

    void EvaluateTriggerExit(Collider other)
    {
    
    }

    Vector3 ProjectDirectionOnPlane(Vector3 direction, Vector3 normal)
    {
        return (direction - normal * Vector3.Dot(direction, normal)).normalized;
    }

    float GetMinDot(int layer)
    {
        return (stairsMask & (1 << layer)) == 0 ?
            minGroundDotProduct : minStairsDotProduct;
    }
}

