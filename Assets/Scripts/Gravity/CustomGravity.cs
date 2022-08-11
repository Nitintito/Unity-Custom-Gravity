using UnityEngine;
using System.Collections.Generic;

public static class CustomGravity
{
    /*
     *This script adds gravitational force to all objects in the gravitySource list.
     */
	static List<GravitySource> sources = new List<GravitySource>();

	public static Vector3 GetGravity(Vector3 position) //Ger possition på gravitations plattsen
	{
		Vector3 g = Vector3.zero;
        for (int i = 0; i < sources.Count; i++)
        {
			g += sources[i].GetGravity(position);
        }
		return g;
	}
	public static Vector3 GetGravity(Vector3 position, out Vector3 upAxis) //ger position och vilket håll som räknas som upp mot gravitationshållet
	{
		Vector3 g = Vector3.zero;
        for (int i = 0; i < sources.Count; i++)
        {
			g += sources[i].GetGravity(position);
        }
		upAxis = -g.normalized;
		return g;
	}

	public static Vector3 GetUpAxis(Vector3 position) //Ger vilket håll som är upp.
	{
		Vector3 g = Vector3.zero;
		for (int i = 0; i < sources.Count; i++)
		{
			g += sources[i].GetGravity(position);
		}
		return -g.normalized;
	}
    public static void Register (GravitySource source) // lägger till gravitationskällor i listan
    {
		Debug.Assert(!sources.Contains(source), "Duplicate registration of gravity source!", source);
		sources.Add(source);
	}
    public static void Unregister(GravitySource source) // tar bort gravitationskällor från listan
    {
		Debug.Assert(sources.Contains(source), "Unregistration of unknown gravity source!", source);
		sources.Remove(source);
	}
}
