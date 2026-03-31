using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(Rigidbody))]
public class AutonomousPhysicsController : MonoBehaviour
{
    // Physics Parameters & Tuning
    [Header("Vehicle Specifications")]
    public float mass = 1500f; // kg
    public float downforceMultiplier = 2.5f;
    public float baseTireFrictionMu = 0.9f; // dry asphalt
    public float centerOfGravityHeight = 0.5f;
    
    [Header("Engine & Brakes")]
    public float maxMotorTorque = 3000f;
    public float maxBrakeTorque = 8000f;
    public float maxSteeringAngle = 35f; // degrees

    [Header("Environment")]
    [Range(0.1f, 1.0f)]
    public float currentFrictionMultiplier = 1.0f; // 1.0 = Dry, 0.6 = Rain

    // AI & Navigation
    [Header("Autonomous Navigation")]
    public Transform targetWaypoint;
    public float userRequestedSpeed = 25f; // m/s (~90 km/h)
    
    private Queue<string> commandBuffer = new Queue<string>();

    // Car State Properties (Telemetry)
    public float CurrentVelocity => rb.velocity.magnitude;
    public float CurrentSlipAngle { get; private set; }
    public float CurrentGForce { get; private set; }
    public float SafeEntrySpeed { get; private set; }

    // Private physics state
    private Rigidbody rb;
    private float currentSteerAngle = 0f;
    private Vector3 lastVelocity;
    
    // Dimensions
    private float wheelbase = 2.8f;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.mass = mass;
        // Lower center of mass to encourage realistic slipping over pure rolling/flipping
        rb.centerOfMass = new Vector3(0, -centerOfGravityHeight, 0); 
    }

    void FixedUpdate()
    {
        ApplyDownforce();
        CalculateTelemetry();
        
        if (targetWaypoint != null)
        {
            ProcessCommand();
            AutonomousDriveLogic();
        }
        
        ApplySuspensionAndTireGrip();
    }

    // --- Core Mechanic: Command Buffer ---
    public void QueueCommand(string command)
    {
        commandBuffer.Enqueue(command);
    }

    private void ProcessCommand()
    {
        // Executes queued user commands
        if (commandBuffer.Count > 0)
        {
            string cmd = commandBuffer.Peek();
            switch (cmd)
            {
                case "Speed Up": userRequestedSpeed += 5f; break;
                case "Slow Down": userRequestedSpeed -= 5f; break;
                case "Emergency Brake": userRequestedSpeed = 0f; break;
            }
            commandBuffer.Dequeue();
        }
    }

    private void AutonomousDriveLogic()
    {
        // 1. Calculate trajectory to target
        Vector3 dirToTarget = (targetWaypoint.position - transform.position).normalized;
        float angleToTarget = Vector3.SignedAngle(transform.forward, dirToTarget, Vector3.up);
        float distanceToTarget = Vector3.Distance(transform.position, targetWaypoint.position);
        
        // 2. Determine turning radius (r) based on chord length approximation
        float turnRadius = distanceToTarget / (2f * Mathf.Sin(Mathf.Abs(angleToTarget) * Mathf.Deg2Rad + 0.001f));
        
        // 3. Calculate Safe Entry Speed
        // max v = sqrt((mu * N * r) / m) => simplified to sqrt(mu * g * r)
        float availableMu = baseTireFrictionMu * currentFrictionMultiplier;
        SafeEntrySpeed = Mathf.Sqrt(availableMu * Physics.gravity.magnitude * turnRadius);

        // 4. Inertia & Braking Calculation
        // Stopping distance s = v^2 / (2 * mu * g)
        float stoppingDistance = (CurrentVelocity * CurrentVelocity) / (2f * availableMu * Physics.gravity.magnitude);
        
        // 5. Throtte & Braking (Wait until latest braking point, maintaining commanded speed)
        float targetSpeed = userRequestedSpeed;

        if (CurrentVelocity < targetSpeed && distanceToTarget > stoppingDistance)
        {
            // Throttle
            rb.AddForce(transform.forward * maxMotorTorque, ForceMode.Force);
        }
        else
        {
            // Brake (triggers weight transfer logic in grip calculation)
            rb.AddForce(-transform.forward * maxBrakeTorque, ForceMode.Force);
            
            // Physical manifestation of weight transfer (nose dipping)
            rb.AddRelativeTorque(Vector3.right * (maxBrakeTorque * 0.5f), ForceMode.Force); 
        }

        // Steer towards target waypoint
        float desiredSteerAngle = Mathf.Clamp(angleToTarget, -maxSteeringAngle, maxSteeringAngle);
        currentSteerAngle = Mathf.Lerp(currentSteerAngle, desiredSteerAngle, Time.fixedDeltaTime * 5f);
    }

    // --- High-Fidelity Physics: Grip & Weight Transfer ---
    private void ApplySuspensionAndTireGrip()
    {
        float mu = baseTireFrictionMu * currentFrictionMultiplier;
        
        // Dynamic Weight Transfer (Acceleration shifts weight backward, braking shifts weight forward)
        float acceleration = (CurrentVelocity - lastVelocity.magnitude) / Time.fixedDeltaTime;
        float weightShift = (acceleration * mass * centerOfGravityHeight) / wheelbase;
        
        // Calculate dynamic normal forces (N) for front and rear
        float normalForceFront = (mass * Physics.gravity.magnitude / 2f) - weightShift;
        float normalForceRear = (mass * Physics.gravity.magnitude / 2f) + weightShift;

        normalForceFront = Mathf.Max(normalForceFront, 0); // clamp
        normalForceRear = Mathf.Max(normalForceRear, 0);

        // Max lateral forces (F_grip = mu * N)
        float maxGripFront = mu * normalForceFront;
        float maxGripRear = mu * normalForceRear;

        // Calculate lateral velocity at axles
        float frontLateralVelocity = Vector3.Dot(rb.velocity, transform.right); // Simplified
        float rearLateralVelocity = Vector3.Dot(rb.velocity, transform.right);
        
        // Apply cornering forces
        float frontGripForce = -frontLateralVelocity * (mass * 0.5f) / Time.fixedDeltaTime;
        float rearGripForce = -rearLateralVelocity * (mass * 0.5f) / Time.fixedDeltaTime;

        // The Fallibility: If steering exceeds grip limit, clamp physical force (inducing slide/drift)
        if (Mathf.Abs(frontGripForce) > maxGripFront)
        {
            frontGripForce = Mathf.Sign(frontGripForce) * maxGripFront; // Understeer
        }

        if (Mathf.Abs(rearGripForce) > maxGripRear)
        {
            rearGripForce = Mathf.Sign(rearGripForce) * maxGripRear; // Oversteer / Drift
        }

        // Apply forces to axles
        rb.AddForceAtPosition(transform.right * frontGripForce, transform.position + transform.forward * (wheelbase / 2));
        rb.AddForceAtPosition(transform.right * rearGripForce, transform.position - transform.forward * (wheelbase / 2));
    }

    private void ApplyDownforce()
    {
        // Aerodynamic downforce (increases with v^2)
        float downforce = rb.velocity.sqrMagnitude * downforceMultiplier;
        rb.AddForce(-transform.up * downforce);
    }

    private void CalculateTelemetry()
    {
        // G-Force
        Vector3 accelerationVector = (rb.velocity - lastVelocity) / Time.fixedDeltaTime;
        CurrentGForce = accelerationVector.magnitude / Physics.gravity.magnitude;
        lastVelocity = rb.velocity;

        // Slip Angle
        if (CurrentVelocity > 1f)
        {
            CurrentSlipAngle = Vector3.SignedAngle(transform.forward, rb.velocity.normalized, Vector3.up);
        }
        else
        {
            CurrentSlipAngle = 0f;
        }
    }

    // --- Weather Events ---
    public void SetWeatherState(bool isRaining)
    {
        // 40% reduction in friction when raining
        currentFrictionMultiplier = isRaining ? 0.6f : 1.0f;
    }

    // --- Telemetry UI Overlay ---
    void OnGUI()
    {
        GUIStyle style = new GUIStyle();
        style.fontSize = 20;
        style.normal.textColor = Color.white;
        style.fontStyle = FontStyle.Bold;

        // Telemetry Box
        GUI.Box(new Rect(10, 10, 350, 180), "VECTOR DRIFT - TELEMETRY");

        GUI.Label(new Rect(20, 40, 300, 30), $"Velocity: {CurrentVelocity * 3.6f:F1} km/h", style);
        GUI.Label(new Rect(20, 70, 300, 30), $"G-Force: {CurrentGForce:F2} G", style);
        
        style.normal.textColor = Mathf.Abs(CurrentSlipAngle) > 10f ? Color.yellow : Color.white;
        GUI.Label(new Rect(20, 100, 300, 30), $"Slip Angle: {CurrentSlipAngle:F1}°", style);
        
        // Highlight danger: If Safe < Velocity, color in red
        style.normal.textColor = SafeEntrySpeed < CurrentVelocity ? Color.red : Color.green;
        GUI.Label(new Rect(20, 130, 300, 30), $"Safe Entry Speed: {SafeEntrySpeed * 3.6f:F1} km/h", style);
        
        style.normal.textColor = currentFrictionMultiplier < 1f ? Color.cyan : Color.white;
        GUI.Label(new Rect(20, 160, 300, 30), $"Env Friction: {(currentFrictionMultiplier * 100):F0}%", style);
    }
}
