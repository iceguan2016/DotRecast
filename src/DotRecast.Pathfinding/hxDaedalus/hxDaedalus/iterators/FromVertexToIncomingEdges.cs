// Generated by Haxe 4.3.6

#pragma warning disable 109, 114, 219, 429, 168, 162
namespace hxDaedalus.iterators {
	public class FromVertexToIncomingEdges : global::haxe.lang.HxObject {
		
		public FromVertexToIncomingEdges(global::haxe.lang.EmptyObject empty) {
		}
		
		
		public FromVertexToIncomingEdges() {
			global::hxDaedalus.iterators.FromVertexToIncomingEdges.__hx_ctor_hxDaedalus_iterators_FromVertexToIncomingEdges(this);
		}
		
		
		protected static void __hx_ctor_hxDaedalus_iterators_FromVertexToIncomingEdges(global::hxDaedalus.iterators.FromVertexToIncomingEdges __hx_this) {
		}
		
		
		
		
		public global::hxDaedalus.data.Vertex _fromVertex;
		
		public global::hxDaedalus.data.Edge _nextEdge;
		
		public global::hxDaedalus.data.Edge _resultEdge;
		
		public virtual global::hxDaedalus.data.Vertex set_fromVertex(global::hxDaedalus.data.Vertex @value) {
			this._fromVertex = @value;
			this._nextEdge = this._fromVertex.get_edge();
			while ( ! (this._nextEdge.get_isReal()) ) {
				this._nextEdge = this._nextEdge.get_rotLeftEdge();
			}
			
			return @value;
		}
		
		
		public virtual global::hxDaedalus.data.Edge next() {
			if (( this._nextEdge != null )) {
				this._resultEdge = this._nextEdge.get_oppositeEdge();
				do {
					this._nextEdge = this._nextEdge.get_rotLeftEdge();
					if (( this._nextEdge == this._fromVertex.get_edge() )) {
						this._nextEdge = null;
						break;
					}
					
				}
				while ( ! (this._nextEdge.get_isReal()) );
			}
			else {
				this._resultEdge = null;
			}
			
			return this._resultEdge;
		}
		
		
		public override object __hx_setField(string field, int hash, object @value, bool handleProperties) {
			unchecked {
				switch (hash) {
					case 114842841:
					{
						this._resultEdge = ((global::hxDaedalus.data.Edge) (@value) );
						return @value;
					}
					
					
					case 913401391:
					{
						this._nextEdge = ((global::hxDaedalus.data.Edge) (@value) );
						return @value;
					}
					
					
					case 924167565:
					{
						this._fromVertex = ((global::hxDaedalus.data.Vertex) (@value) );
						return @value;
					}
					
					
					case 1686888558:
					{
						this.set_fromVertex(((global::hxDaedalus.data.Vertex) (@value) ));
						return @value;
					}
					
					
					default:
					{
						return base.__hx_setField(field, hash, @value, handleProperties);
					}
					
				}
				
			}
		}
		
		
		public override object __hx_getField(string field, int hash, bool throwErrors, bool isCheck, bool handleProperties) {
			unchecked {
				switch (hash) {
					case 1224901875:
					{
						return ((global::haxe.lang.Function) (new global::haxe.lang.Closure(this, "next", 1224901875)) );
					}
					
					
					case 2057392427:
					{
						return ((global::haxe.lang.Function) (new global::haxe.lang.Closure(this, "set_fromVertex", 2057392427)) );
					}
					
					
					case 114842841:
					{
						return this._resultEdge;
					}
					
					
					case 913401391:
					{
						return this._nextEdge;
					}
					
					
					case 924167565:
					{
						return this._fromVertex;
					}
					
					
					default:
					{
						return base.__hx_getField(field, hash, throwErrors, isCheck, handleProperties);
					}
					
				}
				
			}
		}
		
		
		public override object __hx_invokeField(string field, int hash, object[] dynargs) {
			unchecked {
				switch (hash) {
					case 1224901875:
					{
						return this.next();
					}
					
					
					case 2057392427:
					{
						return this.set_fromVertex(((global::hxDaedalus.data.Vertex) (dynargs[0]) ));
					}
					
					
					default:
					{
						return base.__hx_invokeField(field, hash, dynargs);
					}
					
				}
				
			}
		}
		
		
		public override void __hx_getFields(global::HxArray<string> baseArr) {
			baseArr.push("_resultEdge");
			baseArr.push("_nextEdge");
			baseArr.push("_fromVertex");
			baseArr.push("fromVertex");
			base.__hx_getFields(baseArr);
		}
		
		
	}
}


