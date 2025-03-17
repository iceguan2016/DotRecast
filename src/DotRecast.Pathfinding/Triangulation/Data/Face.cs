
namespace Pathfinding.Triangulation.Data
{
    public class Face
    {
        static Face()
        {
            Face.INC = 0;
        }


        public Face()
        {
            colorDebug = -1;

            _id = INC;
            ++INC;
        }

        public static int INC;

        public int _id;

        public bool _isReal;

        public Edge _edge;

        public int colorDebug;

        public int get_id()
        {
            return this._id;
        }


        public bool get_isReal()
        {
            return this._isReal;
        }


        public void set_datas(Edge edge)
        {
            this._isReal = true;
            this._edge = edge;
        }


        public void setDatas(Edge edge, bool isReal = true)
        {
            this._isReal = isReal;
            this._edge = edge;
        }


        public void dispose()
        {
            this._edge = null;
        }


        public Edge get_edge()
        {
            return this._edge;
        }

        public string toString()
        {
            return "Face " + this._id;
        }
    }
}
