
using System.Collections.Generic;

namespace Volatile
{
    // 管理Contact事件通知
    public class ContactInfo
    {
        // Flags stored in m_flags
        public enum ContactFlags
        {
            // Used when crawling contact graph when forming islands.
            e_islandFlag = 0x0001,

            // Set when the shapes are touching.
            e_touchingFlag = 0x0002,

            // This contact can be disabled (by user)
            e_enabledFlag = 0x0004,

            // This contact needs filtering because a fixture filter was changed.
            e_filterFlag = 0x0008,
        };

        /// Is this contact touching?
        public bool IsTouching() 
        {
            int flag = (int)ContactFlags.e_touchingFlag;
            return (m_flags & flag) == flag;
        }

        internal void Update(Manifold manifold, IContactListener listener)
        {
            var touching = manifold != null && manifold.GetContactNum() > 0;
            var touchingFlag = (int)ContactFlags.e_touchingFlag;
            var wasTouching = (m_flags & touchingFlag) == touchingFlag;

            if (touching)
            {
                m_flags |= touchingFlag;
            }
            else
            { 
                m_flags &= ~touchingFlag;
            }

            if (!wasTouching && touching)
            {
                var contact = manifold.GetContact(0);
                position = contact.Position;
                normal = contact.Normal;
                penetration = contact.Penetration;

                if (listener != null) listener.BeginContact(this);
            }

            if (wasTouching && !touching)
            {
                if (listener != null) listener.EndContact(this);
            }
        }

        // hit info
        public VoltVector2 position;
        public VoltVector2 normal;
        public FixMath.NET.Fix64 penetration;

        //
        internal int m_frameNo = 0;
        internal int m_flags = 0;
        internal VoltBody m_bodyA = null;
        internal VoltBody m_bodyB = null;
    }

    internal class ContactManager
    {
        public IContactListener Listener { get; set; }

        private List<ContactInfo> contactInfos = new List<ContactInfo>();
        private IVoltPool<ContactInfo> contactInfoPool;

        private int frameCounter = 1;

        public void UpdateContacts(List<Manifold> manifolds)
        {
            ++frameCounter;
            for (var i = 0; i < manifolds.Count; ++i)
            {
                Manifold manifold = manifolds[i];
                if (manifold != null) 
                {
                    var bodyA = manifold.ShapeA.Body;
                    var bodyB = manifold.ShapeB.Body;
                    var contactIndex = contactInfos.FindIndex(c => {
                        return (c.m_bodyA == bodyA && c.m_bodyB == bodyB) ||
                               (c.m_bodyA == bodyB && c.m_bodyB == bodyA);
                    });
                    //
                    ContactInfo contact = null;
                    if (-1 == contactIndex)
                    {
                        contact = contactInfoPool.Allocate();
                        contact.m_bodyA = bodyA;
                        contact.m_bodyB = bodyB;
                        contactInfos.Add(contact);
                    }
                    else
                    {
                        contact = contactInfos[contactIndex];    
                    }

                    // trigger BeginContact()
                    if (contact.m_bodyA.IsInWorld && contact.m_bodyB.IsInWorld)
                    {
                        contact.m_frameNo = frameCounter;
                        contact.Update(manifold, Listener);
                    }
                    else
                    {
                        contactInfos.RemoveAt(contactIndex);
                    }
                }
            }

            // remove old contact
            for (var i = contactInfos.Count - 1; i >= 0; --i) 
            {
                var contact = contactInfos[i];
                if (contact != null && contact.m_frameNo < frameCounter) 
                {
                    // trigger EndContact()
                    if (contact.m_bodyA.IsInWorld && contact.m_bodyB.IsInWorld)
                    {
                        contact.Update(null, Listener);
                    }
                    contactInfos.RemoveAt(i);
                }
            }
        }
    }
}
