

namespace Pathfinding.Util
{
    public struct UniqueId
    {
        public UniqueId(uint InUniqueId)
        {
            Id = InUniqueId;
            Name = "Unkown";
        }

        public UniqueId(uint InEntityId, string InDebugName)
        {
            Id = InEntityId;
            Name = InDebugName;
        }

        public static UniqueId FromName(string InName)
        {
            return new UniqueId(Crc32.Compute(InName), InName);
        }

        public static UniqueId InvalidID = new UniqueId(0);
        public static bool operator ==(UniqueId v1, UniqueId v2) { return v1.Id == v2.Id; }
        public static bool operator !=(UniqueId v1, UniqueId v2) { return v1.Id != v2.Id; }
        public static bool operator <(UniqueId v1, UniqueId v2) { return v1.Id < v2.Id; }
        public static bool operator <=(UniqueId v1, UniqueId v2) { return v1.Id <= v2.Id; }
        public static bool operator >(UniqueId v1, UniqueId v2) { return v1.Id > v2.Id; }
        public static bool operator >=(UniqueId v1, UniqueId v2) { return v1.Id >= v2.Id; }

        public override bool Equals(object obj)
        {
            if (!(obj is UniqueId))
                return false;
            return ((UniqueId)obj).Id == Id;
        }

        public override int GetHashCode()
        {
            return (int)Id;
        }

        public override string ToString()
        {
            return string.Format("Id:{0}, Name:{1}", Id, Name);
        }

        public uint Id { get; set; }
        public string Name { get; set; }
        public bool IsValid() { return Id > 0; }
    }
}
