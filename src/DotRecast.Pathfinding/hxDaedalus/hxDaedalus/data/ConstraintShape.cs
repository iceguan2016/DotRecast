// Generated by Haxe 4.3.6

#pragma warning disable 109, 114, 219, 429, 168, 162
namespace hxDaedalus.data {
	public class ConstraintShape : global::haxe.lang.HxObject {
		
		static ConstraintShape() {
			global::hxDaedalus.data.ConstraintShape.INC = 0;
		}
		
		
		public ConstraintShape(global::haxe.lang.EmptyObject empty) {
		}
		
		
		public ConstraintShape() {
			global::hxDaedalus.data.ConstraintShape.__hx_ctor_hxDaedalus_data_ConstraintShape(this);
		}
		
		
		protected static void __hx_ctor_hxDaedalus_data_ConstraintShape(global::hxDaedalus.data.ConstraintShape __hx_this) {
			__hx_this._id = global::hxDaedalus.data.ConstraintShape.INC;
			global::hxDaedalus.data.ConstraintShape.INC++;
			__hx_this.segments = new global::HxArray<object>();
		}
		
		
		public static int INC;
		
		
		
		public global::HxArray<object> segments;
		
		public int _id;
		
		public virtual int get_id() {
			return this._id;
		}
		
		
		public virtual void dispose() {
			while (( this.segments.length > 0 )) {
				((global::hxDaedalus.data.ConstraintSegment) ((this.segments.pop()).@value) ).dispose();
			}
			
			this.segments = null;
		}
		
		
		public override double __hx_setField_f(string field, int hash, double @value, bool handleProperties) {
			unchecked {
				switch (hash) {
					case 4747770:
					{
						this._id = ((int) (@value) );
						return @value;
					}
					
					
					default:
					{
						return base.__hx_setField_f(field, hash, @value, handleProperties);
					}
					
				}
				
			}
		}
		
		
		public override object __hx_setField(string field, int hash, object @value, bool handleProperties) {
			unchecked {
				switch (hash) {
					case 4747770:
					{
						this._id = ((int) (global::haxe.lang.Runtime.toInt(@value)) );
						return @value;
					}
					
					
					case 1311173984:
					{
						this.segments = ((global::HxArray<object>) (global::HxArray<object>.__hx_cast<object>(((global::HxArray) (@value) ))) );
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
					case 994869407:
					{
						return ((global::haxe.lang.Function) (new global::haxe.lang.Closure(this, "dispose", 994869407)) );
					}
					
					
					case 590344996:
					{
						return ((global::haxe.lang.Function) (new global::haxe.lang.Closure(this, "get_id", 590344996)) );
					}
					
					
					case 4747770:
					{
						return this._id;
					}
					
					
					case 1311173984:
					{
						return this.segments;
					}
					
					
					case 23515:
					{
						return this.get_id();
					}
					
					
					default:
					{
						return base.__hx_getField(field, hash, throwErrors, isCheck, handleProperties);
					}
					
				}
				
			}
		}
		
		
		public override double __hx_getField_f(string field, int hash, bool throwErrors, bool handleProperties) {
			unchecked {
				switch (hash) {
					case 4747770:
					{
						return ((double) (this._id) );
					}
					
					
					case 23515:
					{
						return ((double) (this.get_id()) );
					}
					
					
					default:
					{
						return base.__hx_getField_f(field, hash, throwErrors, handleProperties);
					}
					
				}
				
			}
		}
		
		
		public override object __hx_invokeField(string field, int hash, object[] dynargs) {
			unchecked {
				switch (hash) {
					case 994869407:
					{
						this.dispose();
						break;
					}
					
					
					case 590344996:
					{
						return this.get_id();
					}
					
					
					default:
					{
						return base.__hx_invokeField(field, hash, dynargs);
					}
					
				}
				
				return null;
			}
		}
		
		
		public override void __hx_getFields(global::HxArray<string> baseArr) {
			baseArr.push("_id");
			baseArr.push("segments");
			baseArr.push("id");
			base.__hx_getFields(baseArr);
		}
		
		
	}
}


