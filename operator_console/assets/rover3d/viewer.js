import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';

const send = value => window.webkit.messageHandlers.rover.postMessage(JSON.stringify(value));
const labels = {front_left:'앞 왼쪽',front_right:'앞 오른쪽',mid_left:'가운데 왼쪽',mid_right:'가운데 오른쪽',rear_left:'뒤 왼쪽',rear_right:'뒤 오른쪽'};
const prefixes = {front_left:'fl',front_right:'fr',mid_left:'cl',mid_right:'cr',rear_left:'rl',rear_right:'rr'};
const suspensionNames = ['ml_rocker_joint','mr_rocker_joint','bl_bogie_joint','br_bogie_joint'];
const armNames=['arm_J1_yaw','arm_J2_shoulder','arm_J3_elbow','arm4_rotation','axis5_rotation'];
const armOnly=new URLSearchParams(location.search).get('view')==='arm';
const armPart=name=>name.startsWith('arm_link')||name==='tool_link'||name.startsWith('finger')||name.startsWith('pinion');
const finite = value => typeof value === 'number' && Number.isFinite(value);
const number = (value, suffix='', digits=2) => finite(value) ? value.toFixed(digits)+suffix : '미연결';
const radians = value => value * Math.PI / 180;
const rpy = (object, value) => object.quaternion.setFromEuler(new THREE.Euler(...value, 'ZYX'));
// Browser page zoom is separate from the fixed-distance 3D camera.
document.addEventListener('wheel',event=>{
    if(event.ctrlKey||event.metaKey)event.preventDefault();
},{capture:true,passive:false});
document.addEventListener('keydown',event=>{
    if((event.ctrlKey||event.metaKey)&&['+','-','=','_','0'].includes(event.key))event.preventDefault();
},{capture:true});
for(const type of ['gesturestart','gesturechange','gestureend'])
    document.addEventListener(type,event=>event.preventDefault(),{capture:true,passive:false});
function smoothCAD(geometry) {
    // Smooth small tessellation seams while retaining real panel/bracket edges.
    const positions=geometry.attributes.position,indices=geometry.index.array;
    const adjacent=Array.from({length:positions.count},()=>[]),faces=new Float32Array(indices.length);
    const a=new THREE.Vector3(),b=new THREE.Vector3(),c=new THREE.Vector3(),normal=new THREE.Vector3();
    for(let i=0;i<indices.length;i+=3){
        a.fromBufferAttribute(positions,indices[i]);b.fromBufferAttribute(positions,indices[i+1]);c.fromBufferAttribute(positions,indices[i+2]);
        normal.crossVectors(b.sub(a),c.sub(a)).normalize();normal.toArray(faces,i);
        for(let j=0;j<3;j++)adjacent[indices[i+j]].push(i);
    }
    const result=geometry.toNonIndexed(),normals=new Float32Array(indices.length*3),cosine=Math.cos(Math.PI/4);
    for(let i=0;i<indices.length;i+=3){
        for(let j=0;j<3;j++){
            let x=0,y=0,z=0;
            for(const other of adjacent[indices[i+j]]){
                if(faces[i]*faces[other]+faces[i+1]*faces[other+1]+faces[i+2]*faces[other+2]<cosine)continue;
                x+=faces[other];y+=faces[other+1];z+=faces[other+2];
            }
            normal.set(x,y,z).normalize().toArray(normals,(i+j)*3);
        }
    }
    result.setAttribute('normal',new THREE.BufferAttribute(normals,3));geometry.dispose();return result;
}
let state = {live:false,wheels:{},suspension:{}};
let lastReceive = 0, selected = null, active = false, frame = null, dirty=true, lastSignature='';
let renderer, scene, robot, camera, controls, fitBox, fitPoints=[];
const links = {}, joints = {}, wheelMeshes = [], motorMeshes=[];
let triangles = 0;

function live() { return state.live && performance.now()-lastReceive <= 1200; }
function armLive() { return state.arm_live && performance.now()-lastReceive <= 1200; }
function row(name) { return live() ? state.wheels[name] : null; }
function setJoint(name, value) {
    if (!finite(value) || !joints[name]) return;
    joints[name].node.quaternion.setFromAxisAngle(joints[name].axis, value);
}
function queueJoint(name,value,duration=120){
    const joint=joints[name];if(!joint||!finite(value))return;
    const target=new THREE.Quaternion().setFromAxisAngle(joint.axis,value);
    if(joint.target&&joint.target.angleTo(target)<1e-7)return;
    if(joint.node.quaternion.angleTo(target)<1e-7)return;
    joint.from=joint.node.quaternion.clone();joint.target=target;joint.started=performance.now();joint.duration=duration;
}
function showPopup(name) {
    if (!(name in labels)) return;
    selected = name;
    document.getElementById('popup').style.display = 'block';
    send({type:'wheel',wheel:name});
    refreshPopup();
    render();
}
function refreshPopup() {
    if (!selected) return;
    const wheel = row(selected);
    const valid = wheel && !wheel.stale;
    document.getElementById('wheel-title').textContent = labels[selected]+' 바퀴';
    const health = !wheel ? '미연결' : wheel.stale ? '갱신 지연' :
        wheel.drive_axis_error || wheel.steer_fault || wheel.mode === 'FAULT' ? '오류' : '정상';
    const values = [
        ['수신 / 모터 상태',health],
        ['모터 전류',number(valid ? wheel.drive_current_a : null,' A')],
        ['회전 속도',number(valid && finite(wheel.drive_turns_per_s) ? wheel.drive_turns_per_s*60 : null,' rpm',0)],
        ['조향각',selected.startsWith('mid_') ? '고정축' : number(valid ? wheel.steer_deg : null,'°',1)],
    ];
    const list=document.getElementById('wheel-values');list.replaceChildren();
    for (const [title,value] of values) {
        const dt=document.createElement('dt'),dd=document.createElement('dd');
        dt.textContent=title;dd.textContent=value;list.append(dt,dd);
    }
}
function refreshStatus() {
    const fresh=live(),link=document.getElementById('link');
    link.textContent=(state.input_source==='TEST'?'더미 자세 · ':'')+(armOnly?'로봇팔 자세':fresh?'차체 상태 수신 중':'차체 상태 대기');
    link.className=fresh?'line':'line warn';
    const armCount=armLive()?armNames.filter(n=>finite(state.arm?.[n])).length:0;
    document.getElementById('attitude').textContent=armCount===5?'로봇팔 자세 수신 중':armCount?'로봇팔 일부 관절 수신':'로봇팔 정보 대기 · 마지막 자세';
    document.getElementById('attitude').className=armCount===5?'line':'line muted';
    const count=fresh?suspensionNames.filter(n=>finite(state.suspension[n])).length:0;
    document.getElementById('suspension').textContent=count===4?'다리 움직임 수신 중':count?'일부 다리 정보 대기':'다리 정보 대기';
    document.getElementById('suspension').className=count===4?'line':'line warn';
    for (const mesh of motorMeshes) {
        const wheel=row(mesh.userData.wheel);
        const color=!wheel?0x969696:wheel.stale?0xffc34a:wheel.drive_axis_error||wheel.steer_fault||wheel.mode==='FAULT'?0xff8a75:0x26e88a;
        mesh.material.color.setHex(color);
        mesh.material.emissive.setHex(mesh.userData.wheel===selected?0x10482d:0x000000);
    }
    refreshPopup();
}
function render() {
    if (renderer && active) {renderer.render(scene,camera);dirty=false;}
}
function setActive(value) {
    value=Boolean(value);if(value===active)return;active=value;
    if (active && frame===null) {previous=performance.now();frame=requestAnimationFrame(tick);}
    if (!active && frame!==null) {cancelAnimationFrame(frame);frame=null;}
    render();
}
let previous=0, lastStatus=0, wasLive=false;
function tick(now) {
    frame=null;
    if (!active) return;
    const elapsed=now-previous;
    if (elapsed>=50) {
        previous=now;
        let moving=false;
        for(const joint of Object.values(joints))if(joint.target){
            const progress=Math.min(1,(now-joint.started)/(joint.duration||120));
            joint.node.quaternion.slerpQuaternions(joint.from,joint.target,progress);
            moving=true;if(progress===1)joint.target=null;
        }
        if (live()) {
            const dt=Math.min(elapsed/1000,.1);
            for (const [name,prefix] of Object.entries(prefixes)) {
                const wheel=row(name),joint=joints[prefix+'_wheel_joint'];
                if (wheel && !wheel.stale && finite(wheel.drive_turns_per_s) && wheel.drive_turns_per_s!==0) {
                    joint.angle=(joint.angle+wheel.drive_turns_per_s*Math.PI*2*dt)%(Math.PI*2);
                    setJoint(prefix+'_wheel_joint',joint.angle);
                    moving=true;
                }
            }
        }
        if (now-lastStatus>250) {const fresh=live();refreshStatus();moving=moving||fresh!==wasLive;wasLive=fresh;lastStatus=now;}
        if(moving||dirty)render();
    }
    frame=requestAnimationFrame(tick);
}
function receive(value) {
    state=value;lastReceive=performance.now();
    if (state.live) {
        if (finite(state.roll_rad)) robot.rotation.x=state.roll_rad;
        if (finite(state.pitch_rad)) robot.rotation.y=state.pitch_rad;
        for (const [name,prefix] of Object.entries(prefixes)) {
            const wheel=state.wheels[name];
            if (wheel && !wheel.stale && finite(wheel.steer_deg)) queueJoint(prefix+'_steer_joint',radians(wheel.steer_deg));
        }
        for (const name of suspensionNames) queueJoint(name,state.suspension[name]);
    }
    if(state.arm_live)for(const name of armNames)queueJoint(name,state.arm[name],260);
    const signature=JSON.stringify([state.live,state.roll_rad,state.pitch_rad,state.suspension,state.arm_live,state.arm,Object.entries(state.wheels).map(([name,w])=>[name,w.stale,w.mode,w.drive_axis_error,w.steer_fault,w.steer_deg])]);
    dirty=dirty||signature!==lastSignature;lastSignature=signature;
    robot.updateMatrixWorld(true);refreshStatus();
}

try {
    renderer=new THREE.WebGLRenderer({antialias:false,preserveDrawingBuffer:true,powerPreference:'low-power'});
    renderer.setPixelRatio(1);
    renderer.setSize(innerWidth,innerHeight);renderer.setClearColor(0x292929);
    document.body.prepend(renderer.domElement);
    scene=new THREE.Scene();robot=new THREE.Group();scene.add(robot);
    camera=new THREE.PerspectiveCamera(38,innerWidth/innerHeight,.01,100);camera.up.set(0,0,1);
    controls=new OrbitControls(camera,renderer.domElement);
    controls.enableZoom=false;controls.enablePan=false;
    controls.minPolarAngle=controls.maxPolarAngle=Math.PI/3;
    controls.mouseButtons={LEFT:THREE.MOUSE.ROTATE,MIDDLE:null,RIGHT:null};
    controls.touches={ONE:THREE.TOUCH.ROTATE,TWO:null};
    function reset() {
        if(!fitBox){camera.position.set(1.6,1.8,1.2);controls.target.set(0,0,-.03);controls.update();return;}
        const sphere=fitBox.getBoundingSphere(new THREE.Sphere());
        const direction=new THREE.Vector3(1.3,1.7,1.0).normalize();
        const right=new THREE.Vector3().crossVectors(direction,new THREE.Vector3(0,0,1)).normalize();
        const up=new THREE.Vector3().crossVectors(right,direction).normalize();
        const tanV=Math.tan(THREE.MathUtils.degToRad(camera.fov/2)),tanH=tanV*camera.aspect;
        let distance=0;
        for(const point of fitPoints){const delta=point.clone().sub(sphere.center);distance=Math.max(distance,Math.abs(delta.dot(right))/tanH+delta.dot(direction),Math.abs(delta.dot(up))/tanV+delta.dot(direction));}
        distance*=1.08;
        controls.target.copy(sphere.center);
        camera.position.copy(sphere.center).add(direction.multiplyScalar(distance));
        controls.minDistance=controls.maxDistance=distance;
        camera.near=Math.max(.001,distance/100);camera.far=distance*50;camera.updateProjectionMatrix();
        controls.update();render();
    }
    reset();
    // A view-relative light rig keeps the reverse side equally readable on orbit.
    // The rover uses Z-up; the hemisphere's default Y-up would tint its sides.
    scene.add(new THREE.AmbientLight(0xffffff,.8));
    const sky=new THREE.HemisphereLight(0xffffff,0x808080,.8);sky.position.set(0,0,1);scene.add(sky);
    scene.add(camera);
    const lightTarget=new THREE.Object3D();lightTarget.position.set(0,0,-1);camera.add(lightTarget);
    const key=new THREE.DirectionalLight(0xffffff,1.3);key.position.set(-3,4,5);key.target=lightTarget;camera.add(key);
    const fill=new THREE.DirectionalLight(0xffffff,.45);fill.position.set(3,-1,2);fill.target=lightTarget;camera.add(fill);
    const model=await(await fetch('./model.json')).json();
    for (const link of model.links) {const node=new THREE.Group();node.name=link.name;links[link.name]=node;}
    const children=new Set(model.joints.map(j=>j.child));
    for (const joint of model.joints) {
        const origin=new THREE.Group(),motion=new THREE.Group();
        origin.position.fromArray(joint.xyz);rpy(origin,joint.rpy);origin.add(motion);
        motion.add(links[joint.child]);links[joint.parent].add(origin);
        joints[joint.name]={node:motion,origin,axis:new THREE.Vector3(...joint.axis).normalize(),angle:0};
    }
    for (const [name,node] of Object.entries(links)) if(!children.has(name)) robot.add(node);
    // Display-only CAD LOD: retain the supplied component shapes and joint frames.
    const display=await(await fetch('./display-mesh-manifest.json')).json();
    if(armOnly)display.meshes=display.meshes.filter(item=>armPart(item.link));
    const selectedMeshes=new Set(display.meshes.map(item=>item.mesh));
    const wheelNames=Object.fromEntries(Object.entries(prefixes).map(([name,prefix])=>[prefix+'_bl70200s',name]));
    const geometryCache=new Map();
    await Promise.all(display.meshes.map(async item=>{
        const response=await fetch('./display-meshes/'+item.mesh);
        if(!response.ok)throw new Error('표시 모델을 읽을 수 없습니다: '+item.mesh);
        const buffer=await response.arrayBuffer(),header=new DataView(buffer);
        const vertices=header.getUint32(0,true),faces=header.getUint32(4,true);
        if(buffer.byteLength!==8+vertices*12+faces*12)throw new Error('표시 모델 형식 오류');
        const geometry=new THREE.BufferGeometry();
        geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(buffer,8,vertices*3),3));
        geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(buffer,8+vertices*12,faces*3),1));
        geometryCache.set(item.mesh,smoothCAD(geometry));
    }));
    for(const link of model.links)for(const visual of link.visuals){
        if(!selectedMeshes.has(visual.mesh))continue;
        const wheel=wheelNames[link.name],geometry=geometryCache.get(visual.mesh);
        let color=armPart(link.name)?(link.name==='arm_link1'||link.name==='arm_link4'?0x959595:0xdddddd):
            link.name==='base_link'?0xa6b8ae:link.name.includes('chassis_pivot')?0x9dafa4:
            link.name.includes('bogie_pivot')?0x96a89d:0xbfbfbf;
        if(wheel){
            const positions=geometry.attributes.position,colors=new Float32Array(positions.count*3);
            const rubber=new THREE.Color(0x303234),rim=new THREE.Color(0xadb0b2);
            for(let i=0;i<positions.count;i++){
                const radius=Math.hypot(positions.getX(i),positions.getZ(i));
                (radius>77?rubber:rim).toArray(colors,i*3);
            }
            geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));color=0xffffff;
        }
        const object=new THREE.Mesh(geometry,new THREE.MeshLambertMaterial({color,vertexColors:Boolean(wheel)}));
        object.position.fromArray(visual.xyz);object.scale.fromArray(visual.scale);rpy(object,visual.rpy);
        if(wheel){object.userData.wheel=wheel;wheelMeshes.push(object);}
        links[link.name].add(object);triangles+=(geometry.index?geometry.index.count:geometry.attributes.position.count)/3;
    }
    // A small colored hub ring conveys motor health without replacing the CAD motor.
    for(const [name,prefix] of Object.entries(armOnly?{}:prefixes)){
        const ring=new THREE.Mesh(new THREE.TorusGeometry(.026,.004,6,20),new THREE.MeshLambertMaterial({color:0x929292}));
        ring.rotation.x=Math.PI/2;ring.position.y=name.endsWith('left')?.050:-.050;
        ring.userData.wheel=name;joints[prefix+'_wheel_joint'].origin.add(ring);
        motorMeshes.push(ring);triangles+=ring.geometry.index.count/3;
    }
    const collectFit=()=>robot.traverse(object=>{
        if(!object.isMesh)return;object.geometry.computeBoundingBox();const b=object.geometry.boundingBox;
        for(const x of [b.min.x,b.max.x])for(const y of [b.min.y,b.max.y])for(const z of [b.min.z,b.max.z])fitPoints.push(new THREE.Vector3(x,y,z).applyMatrix4(object.matrixWorld));
    });
    robot.updateMatrixWorld(true);fitBox=new THREE.Box3().setFromObject(robot);collectFit();
    // Fit a bounded arm motion envelope once; no automatic zoom while moving.
    for(const angles of [[.5,-.7,-1.2,.45,.2],[-.5,-.25,-.6,.2,-.2]]){
        armNames.forEach((name,i)=>setJoint(name,angles[i]));robot.updateMatrixWorld(true);
        fitBox.union(new THREE.Box3().setFromObject(robot));collectFit();
    }
    armNames.forEach(name=>setJoint(name,0));robot.updateMatrixWorld(true);fitBox.expandByScalar(.035);
    reset();
    if(armOnly){document.getElementById('suspension').style.display='none';document.getElementById('footnote').textContent='드래그하여 360° 회전 · 로봇팔 자세';}
    const ray=new THREE.Raycaster();let down=null;
    function pointerDown(event){down=[event.clientX,event.clientY];}
    function pointerUp(event){
        if(!down||Math.hypot(event.clientX-down[0],event.clientY-down[1])>5){down=null;return;}
        down=null;
        const rect=renderer.domElement.getBoundingClientRect();
        ray.setFromCamera(new THREE.Vector2((event.clientX-rect.left)/rect.width*2-1,1-(event.clientY-rect.top)/rect.height*2),camera);
        const hit=ray.intersectObject(robot,true)[0];
        if(hit?.object.userData.wheel)showPopup(hit.object.userData.wheel);
    }
    renderer.domElement.addEventListener('pointerdown',pointerDown);
    renderer.domElement.addEventListener('pointerup',pointerUp);
    renderer.domElement.addEventListener('pointercancel',()=>{down=null;});
    renderer.domElement.addEventListener('wheel',event=>event.preventDefault(),{passive:false});
    controls.addEventListener('change',render);
    document.getElementById('reset').onclick=reset;
    document.getElementById('close').onclick=()=>{selected=null;document.getElementById('popup').style.display='none';refreshStatus();render();};
    window.addEventListener('resize',()=>{camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix();renderer.setSize(innerWidth,innerHeight);reset();render();});
    window.rover3d={receive,setActive,
        rotateView(angle){const offset=camera.position.clone().sub(controls.target);offset.applyAxisAngle(new THREE.Vector3(0,0,1),angle);camera.position.copy(controls.target).add(offset);controls.update();render();},
        performanceCheck(){const gl=renderer.getContext(),times=[];for(let i=0;i<30;i++){const start=performance.now();renderer.render(scene,camera);gl.finish();times.push(performance.now()-start);}times.sort((a,b)=>a-b);return {triangles,drawCalls:renderer.info.render.calls,median_ms:times[15],p95_ms:times[28],pixelRatio:renderer.getPixelRatio(),cadMeshRequests:performance.getEntriesByType('resource').filter(r=>r.name.includes('/meshes/')).length,displayMeshRequests:performance.getEntriesByType('resource').filter(r=>r.name.includes('/display-meshes/')).length};},
        interactionCheck(){
            const before=camera.position.distanceTo(controls.target),distances=[];
            for(const [deltaY,ctrlKey] of [[-1000,false],[1000,false],[-1000,true]]){
                renderer.domElement.dispatchEvent(new WheelEvent('wheel',{deltaY,ctrlKey,bubbles:true,cancelable:true}));
                distances.push(camera.position.distanceTo(controls.target));
            }
            const position=camera.position.clone(),offset=position.clone().sub(controls.target);
            camera.position.copy(controls.target).add(new THREE.Vector3(-offset.x,-offset.y,offset.z));controls.update();
            const halfway=camera.position.clone();camera.position.copy(position);controls.update();
            const pageWheel=new WheelEvent('wheel',{deltaY:-100,ctrlKey:true,bubbles:true,cancelable:true});
            document.getElementById('reset').dispatchEvent(pageWheel);
            const pageKey=new KeyboardEvent('keydown',{key:'+',ctrlKey:true,bubbles:true,cancelable:true});
            document.dispatchEvent(pageKey);
            const pinch=new Event('gesturechange',{bubbles:true,cancelable:true});document.dispatchEvent(pinch);
            return {zoomBefore:before,zoomAfter:distances[0],zoomOutAfter:distances[1],ctrlWheelAfter:distances[2],
                pageWheelBlocked:pageWheel.defaultPrevented,pageKeyBlocked:pageKey.defaultPrevented,pinchBlocked:pinch.defaultPrevented,
                fixedDistance:controls.minDistance===controls.maxDistance,twoFingerDisabled:controls.touches.TWO===null,
                halfTurnDistance:position.distanceTo(halfway),fixedElevation:controls.minPolarAngle===controls.maxPolarAngle};
        },
        inspect(){robot.updateMatrixWorld(true);return {modelKind:'cad-lod',viewMode:armOnly?'arm':'rover',triangles,displayMeshes:display.meshes.length,zoomEnabled:controls.enableZoom,panEnabled:controls.enablePan,selected,live:live(),armLive:Boolean(armLive()),wheels:wheelMeshes.map(m=>m.userData.wheel),joints:Object.fromEntries(suspensionNames.map(n=>[n,joints[n].node.quaternion.toArray()])),armJoints:Object.fromEntries(armNames.map(n=>[n,joints[n].node.quaternion.toArray()])),armTip:links.arm_link5.localToWorld(new THREE.Vector3(.1115,0,0)).toArray(),frontLeft:links.fl_bl70200s.getWorldPosition(new THREE.Vector3()).toArray(),popup:document.getElementById('wheel-values').textContent};},
        selectWheel:showPopup,
        clickWheel(name){const mesh=wheelMeshes.find(m=>m.userData.wheel===name),p=new THREE.Box3().setFromObject(mesh).getCenter(new THREE.Vector3()).project(camera);const e={clientX:(p.x+1)/2*innerWidth,clientY:(1-p.y)/2*innerHeight};pointerDown(e);pointerUp(e);return selected;},
    };
    setActive(true);refreshStatus();
    send({type:'ready',triangles,meshes:wheelMeshes.length,webgl2:renderer.getContext() instanceof WebGL2RenderingContext});
} catch(error) {
    send({type:'error',message:String(error)});
    document.getElementById('link').textContent='3D 표시를 시작할 수 없습니다';
}
