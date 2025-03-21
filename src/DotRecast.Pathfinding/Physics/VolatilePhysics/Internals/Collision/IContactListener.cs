
namespace Volatile
{
    public interface IContactListener
    {
        /// Called when two fixtures begin to touch.
        void BeginContact(ContactInfo contact);

        /// Called when two fixtures cease to touch.
        void EndContact(ContactInfo contact);
    }
}
