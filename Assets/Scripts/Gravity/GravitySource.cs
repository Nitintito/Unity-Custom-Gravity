using UnityEngine;



public class GravitySource : MonoBehaviour
{
    /*  All objects that are supose to affect gravity will enherit this script.
     *  The meaning of this script is to add sources of gravity to the list in CustomGravity script.
     */
    public virtual Vector3 GetGravity (Vector3 position)
    {
        return Physics.gravity;
    }

    void OnEnable() //lägger till objektet i listan
    {
        CustomGravity.Register(this);
    }

    private void OnDisable() //tar bort objektet från lista
    {
        CustomGravity.Unregister(this);
    }
}

