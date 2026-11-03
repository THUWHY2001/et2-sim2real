(() => {
  const radians=degrees=>degrees*Math.PI/180;
  const clamp=(value,min,max)=>Math.min(max,Math.max(min,value));

  function createCamera(options={}){
    const width=Number(options.width)||600,height=Number(options.height)||500;
    const center=options.center||{x:0,y:0,z:430};
    const scale=Number(options.scale)||2;
    const defaults={yaw:-35,pitch:-20,zoom:1};
    let yaw=defaults.yaw,pitch=defaults.pitch,zoom=defaults.zoom;

    function project(point){
      const x=point.x-center.x,y=point.y-center.y,z=point.z-center.z;
      const a=radians(yaw),ca=Math.cos(a),sa=Math.sin(a);
      const xr=x*ca-y*sa,yr=x*sa+y*ca;
      const b=radians(pitch),cb=Math.cos(b),sb=Math.sin(b);
      const depth=yr*cb-z*sb,zr=yr*sb+z*cb;
      return{x:width/2+xr*scale*zoom,y:height/2-zr*scale*zoom,depth};
    }

    return{
      rotate(dx,dy){yaw+=Number(dx||0)*.45;pitch=clamp(pitch+Number(dy||0)*.45,-85,85);return this},
      zoom(delta){zoom=clamp(zoom+Number(delta||0)*.02,.55,2.4);return this},
      reset(){yaw=defaults.yaw;pitch=defaults.pitch;zoom=defaults.zoom;return this},
      project,
      state(){return{yaw,pitch,zoom}}
    };
  }

  window.Neck3DView={createCamera};
})();
