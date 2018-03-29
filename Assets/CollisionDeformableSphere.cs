using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionDeformableSphere : MonoBehaviour {

	public class CollisionInfo{
		public Collision collision = null;
		public bool resolved = true;

		public CollisionInfo(Collision collision){
			this.collision = collision;
		}
	}

	public MeshFilter filter = null;
	public Mesh deformingMesh = null;
	Vector3[] originalVerts, displacedVerts;
	public int vertCount = 0;
	public float forceOffset = 0.1f;


	Transform myTrans = null;

	Vector3[] vertVels;

	public float massPerVert = 1;

	public float springForce = 20;

	public float damping = 5;

	public float maxDisplacement = 50;

	public float forceAbsorption = 1;


	public Rigidbody rb = null;

	Dictionary<Collider, CollisionInfo> stayingCollisionsByCollider = new Dictionary<Collider, CollisionInfo> ();
	HashSet<Collision> stayingCollisions = new HashSet<Collision> ();

	Vector3 originalCenter = Vector3.zero;

	float originalDistanceToCenterSum = 0;

	SphereCollider sCol = null;
	CapsuleCollider cCol = null;

	// Use this for initialization
	void Start () {
		myTrans = transform;
		if (this.filter == null) {
			this.filter = GetComponent<MeshFilter> ();
		}
		if (this.deformingMesh == null && this.filter != null) {
			this.deformingMesh = this.filter.mesh;
		}
		if (this.deformingMesh == null) {
			this.enabled = false;
		} else {
			this.originalVerts = deformingMesh.vertices;
			vertCount = originalVerts.Length;
			this.displacedVerts = new Vector3[vertCount];
			vertVels = new Vector3[vertCount];
			for (int i = 0; i < vertCount; ++i) {
				displacedVerts [i] = originalVerts [i];
				vertVels [i] = Vector3.zero;
				originalCenter += originalVerts [i];
			}
			originalCenter /= vertCount;
			for (int i = 0; i < vertCount; ++i) {
				originalDistanceToCenterSum += Vector3.Distance (originalVerts [i], originalCenter);
			}
			this.rb = GetComponent<Rigidbody> ();
			this.sCol = GetComponent<SphereCollider> ();
			this.cCol = GetComponent<CapsuleCollider> ();
		}
	}

	Vector3[] noResult = new Vector3[]{ Vector3.zero, Vector3.zero };

	public Vector3[] AddDeformingForce(Vector3 position, Vector3 force, Vector3 normal){

		if (force.magnitude == 0) {
			Debug.Log ("Magnitude 0");
			return noResult;
		}

		Vector3 appliedForce30 = Vector3.Project (force * forceAbsorption, normal);
		Vector3 remainedForce = force - appliedForce30;

		position = position + normal * forceOffset;

		Debug.DrawLine (position, force, Color.white);
		Debug.DrawLine (position, appliedForce30, Color.red);
		Debug.DrawLine (position, remainedForce, Color.green);

		Vector3 appliedForce3 = myTrans.InverseTransformVector (appliedForce30);
		float appliedForce = appliedForce3.magnitude;
		position = myTrans.InverseTransformPoint (position);

		for (int i = 0; i < vertCount; ++i) {
			AddForceToVert (i, position, appliedForce);
		}

		return new Vector3[]{ appliedForce30, remainedForce };
	}


	void AddForceToVert(int i, Vector3 position, float force){
		if (force == 0) {
			Debug.Log ("Magnitude 0");
			return;
		}
		Vector3 posToVert = displacedVerts [i] - position;
		float attenuatedForce = force / (1f + posToVert.sqrMagnitude);
		float vel = attenuatedForce * Time.fixedDeltaTime / massPerVert;
		vertVels [i] += posToVert.normalized * vel;
	}


	public bool reset = false;

	void Reset(){
		for (int i = 0; i < vertCount; ++i) {
			displacedVerts [i] = originalVerts [i];
			vertVels [i] = Vector3.zero;
		}
		reset = false;
	}


	void FixedUpdate () {

		if (reset) {
			Reset ();
		}

		foreach (Collision col in stayingCollisions) {
			Debug.Log ("CollisionStay impulse " + col.impulse);
			ResolveCollision (col);
		}

		for (int i = 0; i < vertCount; i++) {
			UpdateVertex(i);
		}

		Vector3 center = Vector3.zero;

		for (int i = 0; i < vertCount; ++i) {
			center += displacedVerts [i];
		}
		center /= vertCount;
		float distanceToCenterSum = 0;
		float smallestDistanceToCenter = float.PositiveInfinity;
		for (int i = 0; i < vertCount; ++i) {
			displacedVerts[i] -= center;
			float dist = displacedVerts [i].magnitude;
			distanceToCenterSum += dist;
			if (smallestDistanceToCenter > dist) {
				smallestDistanceToCenter = dist;
			}
		}

		SetRadius(smallestDistanceToCenter);

		float scale = originalDistanceToCenterSum / distanceToCenterSum;

		for (int i = 0; i < vertCount; ++i) {
			displacedVerts [i] *= scale;
		}

		deformingMesh.vertices = displacedVerts;
		deformingMesh.RecalculateNormals();


	}

	void SetRadius(float radius){
		if (this.cCol) {
			this.cCol.radius = radius;
			this.cCol.height = 2 * radius;
		} else if (this.sCol) {
			this.sCol.radius = radius;
		}
	}

	void UpdateVertex(int i){

		Vector3 vel = vertVels [i];

		Vector3 displacement = displacedVerts[i] - originalVerts[i];
		vel -= displacement * springForce * Time.fixedDeltaTime;
		vel *= 1f - damping * Time.fixedDeltaTime;
		vertVels[i] = vel;

		displacedVerts [i] += vertVels [i] * Time.fixedDeltaTime;

		displacement = displacedVerts [i] - originalVerts [i];
		displacement = Vector3.ClampMagnitude (displacement, maxDisplacement);

		displacedVerts [i] = originalVerts [i] + displacement;
	}

	public void ResolveCollision (Collision col){
		if (col.impulse.magnitude == 0) {
			Debug.Log ("Magnitude 0");
			return;
		}

		ContactPoint[] contacts0 = col.contacts;
		int contactCount0 = contacts0.Length;
		int contactCount = 0;
		ContactPoint[] contacts1 = new ContactPoint[contactCount0];

		for (int i = 0; i < contactCount0; ++i) {
			if (float.IsNaN (contacts0 [i].point.x) ||
				float.IsNaN (contacts0 [i].point.y) ||
				float.IsNaN (contacts0 [i].point.z) ||
				float.IsNaN (contacts0 [i].normal.x) ||
				float.IsNaN (contacts0 [i].normal.y) ||
				float.IsNaN (contacts0 [i].normal.z)) {
				continue;
			}
			contacts1 [contactCount] = contacts0 [i];
			++contactCount;
		}

		if (contactCount == 0) {
			return;
		}


		ContactPoint[] contacts = new ContactPoint[contactCount];
		for (int i = 0; i < contactCount; ++i) {
			contacts [i] = contacts1 [i];
		}
		Vector3 force = -col.impulse / Time.fixedDeltaTime;

		/*
		float projectionSum = 0;
		float[] projectionLengths = new float[contactCount];
		for (int i = 0; i < contactCount; ++i) {
				
			projectionLengths[i] = Vector3.Project (force, contacts [i].normal).magnitude;
			projectionSum += projectionLengths [i];
		}
		for (int i = 0; i < contactCount; ++i) {
			Vector3[] res = AddDeformingForce (contacts [i].point, force * projectionLengths[i] / projectionSum, -contacts [i].normal);
			if (rb) {
				rb.AddForceAtPosition (contacts [i].point, -res [0], ForceMode.Force);
			}
		}
		*/

		Vector3 meanPosition = Vector3.zero;
		Vector3 meanNormal = Vector3.zero;

		for (int i = 0; i < contactCount; ++i) {
			meanPosition += contacts [i].point;
			meanNormal += contacts [i].normal;
		}

		meanPosition /= contactCount;
		meanNormal /= contactCount;

		/*Vector3 meanPosition = contacts [0].point;
		Vector3 meanNormal = contacts [0].normal;*/


		Vector3[] res = AddDeformingForce (meanPosition, force, -meanNormal);
		if (rb) {
			rb.AddForceAtPosition (meanPosition, -res [0], ForceMode.Force);
		}

		

	}

	void OnCollisionEnter(Collision col){
		Debug.Log ("CollisionEnter impulse " + col.impulse);
		ResolveCollision (col);
	}

	void OnCollisionStay(Collision col){
		if (stayingCollisionsByCollider.ContainsKey (col.collider)) {
			stayingCollisionsByCollider [col.collider].resolved = true;
			stayingCollisions.Remove (col);
			stayingCollisionsByCollider [col.collider].collision = col;
		} else {
			stayingCollisionsByCollider [col.collider] = new CollisionInfo(col);
			stayingCollisions.Add (col);
		}
	}

	void OnCollisionExit(Collision col){
		Debug.Log ("CollisionExit impulse " + col.impulse);
		ResolveCollision (col);
		if (stayingCollisionsByCollider.ContainsKey (col.collider)) {
			stayingCollisions.Remove (stayingCollisionsByCollider [col.collider].collision);
			stayingCollisionsByCollider.Remove (col.collider);
		}
	}
}
