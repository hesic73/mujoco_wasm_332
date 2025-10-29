// Type definitions for Emscripten 1.39.16
// Project: https://emscripten.org
// Definitions by: Kensuke Matsuzaki <https://github.com/zakki>
//                 Periklis Tsirakidis <https://github.com/periklis>
//                 Bumsik Kim <https://github.com/kbumsik>
//                 Louis DeScioli <https://github.com/lourd>
// Definitions: https://github.com/DefinitelyTyped/DefinitelyTyped/blob/master/types/emscripten/index.d.ts
// TypeScript Version: 2.2

/** Other WebAssembly declarations, for compatibility with older versions of Typescript */
declare namespace WebAssembly {
    interface Module {}
}

declare namespace Emscripten {
  interface FileSystemType {}
  type EnvironmentType = 'WEB' | 'NODE' | 'SHELL' | 'WORKER';

  type JSType = 'number' | 'string' | 'array' | 'boolean';
  type TypeCompatibleWithC = number | string | any[] | boolean;

  type CIntType = 'i8' | 'i16' | 'i32' | 'i64';
  type CFloatType = 'float' | 'double';
  type CPointerType = 'i8*' | 'i16*' | 'i32*' | 'i64*' | 'float*' | 'double*' | '*';
  type CType = CIntType | CFloatType | CPointerType;

  type WebAssemblyImports = Array<{
      name: string;
      kind: string;
  }>;

  type WebAssemblyExports = Array<{
      module: string;
      name: string;
      kind: string;
  }>;

  interface CCallOpts {
      async?: boolean | undefined;
  }
}

interface EmscriptenModule {
  print(str: string): void;
  printErr(str: string): void;
  arguments: string[];
  environment: Emscripten.EnvironmentType;
  preInit: Array<{ (): void }>;
  preRun: Array<{ (): void }>;
  postRun: Array<{ (): void }>;
  onAbort: { (what: any): void };
  onRuntimeInitialized: { (): void };
  preinitializedWebGLContext: WebGLRenderingContext;
  noInitialRun: boolean;
  noExitRuntime: boolean;
  logReadFiles: boolean;
  filePackagePrefixURL: string;
  wasmBinary: ArrayBuffer;

  destroy(object: object): void;
  getPreloadedPackage(remotePackageName: string, remotePackageSize: number): ArrayBuffer;
  instantiateWasm(
      imports: Emscripten.WebAssemblyImports,
      successCallback: (module: WebAssembly.Module) => void,
  ): Emscripten.WebAssemblyExports;
  locateFile(url: string, scriptDirectory: string): string;
  onCustomMessage(event: MessageEvent): void;

  // USE_TYPED_ARRAYS == 1
  HEAP: Int32Array;
  IHEAP: Int32Array;
  FHEAP: Float64Array;

  // USE_TYPED_ARRAYS == 2
  HEAP8: Int8Array;
  HEAP16: Int16Array;
  HEAP32: Int32Array;
  HEAPU8: Uint8Array;
  HEAPU16: Uint16Array;
  HEAPU32: Uint32Array;
  HEAPF32: Float32Array;
  HEAPF64: Float64Array;

  TOTAL_STACK: number;
  TOTAL_MEMORY: number;
  FAST_MEMORY: number;

  addOnPreRun(cb: () => any): void;
  addOnInit(cb: () => any): void;
  addOnPreMain(cb: () => any): void;
  addOnExit(cb: () => any): void;
  addOnPostRun(cb: () => any): void;

  preloadedImages: any;
  preloadedAudios: any;

  _malloc(size: number): number;
  _free(ptr: number): void;
}

/**
* A factory function is generated when setting the `MODULARIZE` build option
* to `1` in your Emscripten build. It return a Promise that resolves to an
* initialized, ready-to-call `EmscriptenModule` instance.
*
* By default, the factory function will be named `Module`. It's recommended to
* use the `EXPORT_ES6` option, in which the factory function will be the
* default export. If used without `EXPORT_ES6`, the factory function will be a
* global variable. You can rename the variable using the `EXPORT_NAME` build
* option. It's left to you to declare any global variables as needed in your
* application's types.
* @param moduleOverrides Default properties for the initialized module.
*/
type EmscriptenModuleFactory<T extends EmscriptenModule = EmscriptenModule> = (
  moduleOverrides?: Partial<T>,
) => Promise<T>;

declare namespace FS {
  interface Lookup {
      path: string;
      node: FSNode;
  }

  interface FSStream {}
  interface FSNode {}
  interface ErrnoError {}

  let ignorePermissions: boolean;
  let trackingDelegate: any;
  let tracking: any;
  let genericErrors: any;

  //
  // paths
  //
  function lookupPath(path: string, opts: any): Lookup;
  function getPath(node: FSNode): string;

  //
  // nodes
  //
  function isFile(mode: number): boolean;
  function isDir(mode: number): boolean;
  function isLink(mode: number): boolean;
  function isChrdev(mode: number): boolean;
  function isBlkdev(mode: number): boolean;
  function isFIFO(mode: number): boolean;
  function isSocket(mode: number): boolean;

  //
  // devices
  //
  function major(dev: number): number;
  function minor(dev: number): number;
  function makedev(ma: number, mi: number): number;
  function registerDevice(dev: number, ops: any): void;

  //
  // core
  //
  function syncfs(populate: boolean, callback: (e: any) => any): void;
  function syncfs(callback: (e: any) => any, populate?: boolean): void;
  function mount(type: Emscripten.FileSystemType, opts: any, mountpoint: string): any;
  function unmount(mountpoint: string): void;

  function mkdir(path: string, mode?: number): any;
  function mkdev(path: string, mode?: number, dev?: number): any;
  function symlink(oldpath: string, newpath: string): any;
  function rename(old_path: string, new_path: string): void;
  function rmdir(path: string): void;
  function readdir(path: string): any;
  function unlink(path: string): void;
  function readlink(path: string): string;
  function stat(path: string, dontFollow?: boolean): any;
  function lstat(path: string): any;
  function chmod(path: string, mode: number, dontFollow?: boolean): void;
  function lchmod(path: string, mode: number): void;
  function fchmod(fd: number, mode: number): void;
  function chown(path: string, uid: number, gid: number, dontFollow?: boolean): void;
  function lchown(path: string, uid: number, gid: number): void;
  function fchown(fd: number, uid: number, gid: number): void;
  function truncate(path: string, len: number): void;
  function ftruncate(fd: number, len: number): void;
  function utime(path: string, atime: number, mtime: number): void;
  function open(path: string, flags: string, mode?: number, fd_start?: number, fd_end?: number): FSStream;
  function close(stream: FSStream): void;
  function llseek(stream: FSStream, offset: number, whence: number): any;
  function read(stream: FSStream, buffer: ArrayBufferView, offset: number, length: number, position?: number): number;
  function write(
      stream: FSStream,
      buffer: ArrayBufferView,
      offset: number,
      length: number,
      position?: number,
      canOwn?: boolean,
  ): number;
  function allocate(stream: FSStream, offset: number, length: number): void;
  function mmap(
      stream: FSStream,
      buffer: ArrayBufferView,
      offset: number,
      length: number,
      position: number,
      prot: number,
      flags: number,
  ): any;
  function ioctl(stream: FSStream, cmd: any, arg: any): any;
  function readFile(path: string, opts: { encoding: 'binary'; flags?: string | undefined }): Uint8Array;
  function readFile(path: string, opts: { encoding: 'utf8'; flags?: string | undefined }): string;
  function readFile(path: string, opts?: { flags?: string | undefined }): Uint8Array;
  function writeFile(path: string, data: string | ArrayBufferView, opts?: { flags?: string | undefined }): void;

  //
  // module-level FS code
  //
  function cwd(): string;
  function chdir(path: string): void;
  function init(
      input: null | (() => number | null),
      output: null | ((c: number) => any),
      error: null | ((c: number) => any),
  ): void;

  function createLazyFile(
      parent: string | FSNode,
      name: string,
      url: string,
      canRead: boolean,
      canWrite: boolean,
  ): FSNode;
  function createPreloadedFile(
      parent: string | FSNode,
      name: string,
      url: string,
      canRead: boolean,
      canWrite: boolean,
      onload?: () => void,
      onerror?: () => void,
      dontCreateFile?: boolean,
      canOwn?: boolean,
  ): void;
  function createDataFile(
      parent: string | FSNode,
      name: string,
      data: ArrayBufferView,
      canRead: boolean,
      canWrite: boolean,
      canOwn: boolean,
  ): FSNode;
}

declare var MEMFS: Emscripten.FileSystemType;
declare var NODEFS: Emscripten.FileSystemType;
declare var IDBFS: Emscripten.FileSystemType;

// https://emscripten.org/docs/porting/connecting_cpp_and_javascript/Interacting-with-code.html
type StringToType<R extends any> = R extends Emscripten.JSType
? {
    number: number;
    string: string;
    array: number[] | string[] | boolean[] | Uint8Array | Int8Array;
    boolean: boolean;
    null: null;
  }[R]
: never;

type ArgsToType<T extends Array<Emscripten.JSType | null>> = Extract<
{
  [P in keyof T]: StringToType<T[P]>;
},
any[]
>;

type ReturnToType<R extends Emscripten.JSType | null> = R extends null
? null
: StringToType<Exclude<R, null>>;

// ENUMS
/**  disable default feature bitflags        */
export enum mjtDisableBit {
    /** entire constraint solver                 */
    mjDSBL_CONSTRAINT        ,
    /** equality constraints                     */
    mjDSBL_EQUALITY          ,
    /** joint and tendon frictionloss constraints */
    mjDSBL_FRICTIONLOSS      ,
    /** joint and tendon limit constraints       */
    mjDSBL_LIMIT             ,
    /** contact constraints                      */
    mjDSBL_CONTACT           ,
    /** passive forces                           */
    mjDSBL_PASSIVE           ,
    /** gravitational forces                     */
    mjDSBL_GRAVITY           ,
    /** clamp control to specified range         */
    mjDSBL_CLAMPCTRL         ,
    /** warmstart constraint solver              */
    mjDSBL_WARMSTART         ,
    /** remove collisions with parent body       */
    mjDSBL_FILTERPARENT      ,
    /** apply actuation forces                   */
    mjDSBL_ACTUATION         ,
    /** integrator safety: make ref[0]>=2*timestep */
    mjDSBL_REFSAFE           ,
    /** sensors                                  */
    mjDSBL_SENSOR            ,
    /** mid-phase collision filtering            */
    mjDSBL_MIDPHASE          ,
    /** implicit integration of joint damping in Euler integrator */
    mjDSBL_EULERDAMP         ,
    /** automatic reset when numerical issues are detected */
    mjDSBL_AUTORESET         ,
    /** native convex collision detection        */
    mjDSBL_NATIVECCD         ,
    /** number of disable flags                  */
    mjNDISABLE               ,
}
/**  enable optional feature bitflags        */
export enum mjtEnableBit {
    /** override contact parameters              */
    mjENBL_OVERRIDE          ,
    /** energy computation                       */
    mjENBL_ENERGY            ,
    /** record solver statistics                 */
    mjENBL_FWDINV            ,
    /** discrete-time inverse dynamics           */
    mjENBL_INVDISCRETE       ,
    /** multi-point convex collision detection   */
    mjENBL_MULTICCD          ,
    /** constraint island discovery              */
    mjENBL_ISLAND            ,
    /** number of enable flags                   */
    mjNENABLE                ,
}
/**  type of degree of freedom               */
export enum mjtJoint {
    /** global position and orientation (quat)       (7) */
    mjJNT_FREE               ,
    /** orientation (quat) relative to parent        (4) */
    mjJNT_BALL               ,
    /** sliding distance along body-fixed axis       (1) */
    mjJNT_SLIDE              ,
    /** rotation angle (rad) around body-fixed axis  (1) */
    mjJNT_HINGE              ,
}
/**  type of geometric shape                 */
export enum mjtGeom {
    /** plane                                    */
    mjGEOM_PLANE             ,
    /** height field                             */
    mjGEOM_HFIELD            ,
    /** sphere                                   */
    mjGEOM_SPHERE            ,
    /** capsule                                  */
    mjGEOM_CAPSULE           ,
    /** ellipsoid                                */
    mjGEOM_ELLIPSOID         ,
    /** cylinder                                 */
    mjGEOM_CYLINDER          ,
    /** box                                      */
    mjGEOM_BOX               ,
    /** mesh                                     */
    mjGEOM_MESH              ,
    /** signed distance field                    */
    mjGEOM_SDF               ,
    /** number of regular geom types             */
    mjNGEOMTYPES             ,
    /** arrow                                    */
    mjGEOM_ARROW             ,
    /** arrow without wedges                     */
    mjGEOM_ARROW1            ,
    /** arrow in both directions                 */
    mjGEOM_ARROW2            ,
    /** line                                     */
    mjGEOM_LINE              ,
    /** box with line edges                      */
    mjGEOM_LINEBOX           ,
    /** flex                                     */
    mjGEOM_FLEX              ,
    /** skin                                     */
    mjGEOM_SKIN              ,
    /** text label                               */
    mjGEOM_LABEL             ,
    /** triangle                                 */
    mjGEOM_TRIANGLE          ,
    /** missing geom type                        */
    mjGEOM_NONE              ,
}
/**  tracking mode for camera and light      */
export enum mjtCamLight {
    /** pos and rot fixed in body                */
    mjCAMLIGHT_FIXED         ,
    /** pos tracks body, rot fixed in global     */
    mjCAMLIGHT_TRACK         ,
    /** pos tracks subtree com, rot fixed in body */
    mjCAMLIGHT_TRACKCOM      ,
    /** pos fixed in body, rot tracks target body */
    mjCAMLIGHT_TARGETBODY    ,
    /** pos fixed in body, rot tracks target subtree com */
    mjCAMLIGHT_TARGETBODYCOM ,
}
/**  type of texture                         */
export enum mjtTexture {
    /** 2d texture, suitable for planes and hfields */
    mjTEXTURE_2D             ,
    /** cube texture, suitable for all other geom types */
    mjTEXTURE_CUBE           ,
    /** cube texture used as skybox              */
    mjTEXTURE_SKYBOX         ,
}
/**  role of texture map in rendering        */
export enum mjtTextureRole {
    /** unspecified                              */
    mjTEXROLE_USER           ,
    /** base color (albedo)                      */
    mjTEXROLE_RGB            ,
    /** ambient occlusion                        */
    mjTEXROLE_OCCLUSION      ,
    /** roughness                                */
    mjTEXROLE_ROUGHNESS      ,
    /** metallic                                 */
    mjTEXROLE_METALLIC       ,
    /** normal (bump) map                        */
    mjTEXROLE_NORMAL         ,
    /** transperancy                             */
    mjTEXROLE_OPACITY        ,
    /** light emission                           */
    mjTEXROLE_EMISSIVE       ,
    /** base color, opacity                      */
    mjTEXROLE_RGBA           ,
    /** occlusion, roughness, metallic           */
    mjTEXROLE_ORM            ,
    mjNTEXROLE               ,
}
/**  integrator mode                         */
export enum mjtIntegrator {
    /** semi-implicit Euler                      */
    mjINT_EULER              ,
    /** 4th-order Runge Kutta                    */
    mjINT_RK4                ,
    /** implicit in velocity                     */
    mjINT_IMPLICIT           ,
    /** implicit in velocity, no rne derivative  */
    mjINT_IMPLICITFAST       ,
}
/**  type of friction cone                   */
export enum mjtCone {
    /** pyramidal                                */
    mjCONE_PYRAMIDAL         ,
    /** elliptic                                 */
    mjCONE_ELLIPTIC          ,
}
/**  type of constraint Jacobian             */
export enum mjtJacobian {
    /** dense                                    */
    mjJAC_DENSE              ,
    /** sparse                                   */
    mjJAC_SPARSE             ,
    /** dense if nv<60, sparse otherwise         */
    mjJAC_AUTO               ,
}
/**  constraint solver algorithm             */
export enum mjtSolver {
    /** PGS    (dual)                            */
    mjSOL_PGS                ,
    /** CG     (primal)                          */
    mjSOL_CG                 ,
    /** Newton (primal)                          */
    mjSOL_NEWTON             ,
}
/**  type of equality constraint             */
export enum mjtEq {
    /** connect two bodies at a point (ball joint) */
    mjEQ_CONNECT             ,
    /** fix relative position and orientation of two bodies */
    mjEQ_WELD                ,
    /** couple the values of two scalar joints with cubic */
    mjEQ_JOINT               ,
    /** couple the lengths of two tendons with cubic */
    mjEQ_TENDON              ,
    /** fix all edge lengths of a flex           */
    mjEQ_FLEX                ,
    /** unsupported, will cause an error if used */
    mjEQ_DISTANCE            ,
}
/**  type of tendon wrap object              */
export enum mjtWrap {
    /** null object                              */
    mjWRAP_NONE              ,
    /** constant moment arm                      */
    mjWRAP_JOINT             ,
    /** pulley used to split tendon              */
    mjWRAP_PULLEY            ,
    /** pass through site                        */
    mjWRAP_SITE              ,
    /** wrap around sphere                       */
    mjWRAP_SPHERE            ,
    /** wrap around (infinite) cylinder          */
    mjWRAP_CYLINDER          ,
}
/**  type of actuator transmission           */
export enum mjtTrn {
    /** force on joint                           */
    mjTRN_JOINT              ,
    /** force on joint, expressed in parent frame */
    mjTRN_JOINTINPARENT      ,
    /** force via slider-crank linkage           */
    mjTRN_SLIDERCRANK        ,
    /** force on tendon                          */
    mjTRN_TENDON             ,
    /** force on site                            */
    mjTRN_SITE               ,
    /** adhesion force on a body's geoms         */
    mjTRN_BODY               ,
    /** undefined transmission type              */
    mjTRN_UNDEFINED          ,
}
/**  type of actuator dynamics               */
export enum mjtDyn {
    /** no internal dynamics; ctrl specifies force */
    mjDYN_NONE               ,
    /** integrator: da/dt = u                    */
    mjDYN_INTEGRATOR         ,
    /** linear filter: da/dt = (u-a) / tau       */
    mjDYN_FILTER             ,
    /** linear filter: da/dt = (u-a) / tau, with exact integration */
    mjDYN_FILTEREXACT        ,
    /** piece-wise linear filter with two time constants */
    mjDYN_MUSCLE             ,
    /** user-defined dynamics type               */
    mjDYN_USER               ,
}
/**  type of actuator gain                   */
export enum mjtGain {
    /** fixed gain                               */
    mjGAIN_FIXED             ,
    /** const + kp*length + kv*velocity          */
    mjGAIN_AFFINE            ,
    /** muscle FLV curve computed by mju_muscleGain() */
    mjGAIN_MUSCLE            ,
    /** user-defined gain type                   */
    mjGAIN_USER              ,
}
/**  type of actuator bias                   */
export enum mjtBias {
    /** no bias                                  */
    mjBIAS_NONE              ,
    /** const + kp*length + kv*velocity          */
    mjBIAS_AFFINE            ,
    /** muscle passive force computed by mju_muscleBias() */
    mjBIAS_MUSCLE            ,
    /** user-defined bias type                   */
    mjBIAS_USER              ,
}
/**  type of MujoCo object                   */
export enum mjtObj {
    /** unknown object type                      */
    mjOBJ_UNKNOWN            ,
    /** body                                     */
    mjOBJ_BODY               ,
    /** body, used to access regular frame instead of i-frame */
    mjOBJ_XBODY              ,
    /** joint                                    */
    mjOBJ_JOINT              ,
    /** dof                                      */
    mjOBJ_DOF                ,
    /** geom                                     */
    mjOBJ_GEOM               ,
    /** site                                     */
    mjOBJ_SITE               ,
    /** camera                                   */
    mjOBJ_CAMERA             ,
    /** light                                    */
    mjOBJ_LIGHT              ,
    /** flex                                     */
    mjOBJ_FLEX               ,
    /** mesh                                     */
    mjOBJ_MESH               ,
    /** skin                                     */
    mjOBJ_SKIN               ,
    /** heightfield                              */
    mjOBJ_HFIELD             ,
    /** texture                                  */
    mjOBJ_TEXTURE            ,
    /** material for rendering                   */
    mjOBJ_MATERIAL           ,
    /** geom pair to include                     */
    mjOBJ_PAIR               ,
    /** body pair to exclude                     */
    mjOBJ_EXCLUDE            ,
    /** equality constraint                      */
    mjOBJ_EQUALITY           ,
    /** tendon                                   */
    mjOBJ_TENDON             ,
    /** actuator                                 */
    mjOBJ_ACTUATOR           ,
    /** sensor                                   */
    mjOBJ_SENSOR             ,
    /** numeric                                  */
    mjOBJ_NUMERIC            ,
    /** text                                     */
    mjOBJ_TEXT               ,
    /** tuple                                    */
    mjOBJ_TUPLE              ,
    /** keyframe                                 */
    mjOBJ_KEY                ,
    /** plugin instance                          */
    mjOBJ_PLUGIN             ,
    /** number of object types                   */
    mjNOBJECT                ,
    /** frame                                    */
    mjOBJ_FRAME              ,
    /** default                                  */
    mjOBJ_DEFAULT            ,
    /** entire model                             */
    mjOBJ_MODEL              ,
}
/**  type of constraint                      */
export enum mjtConstraint {
    /** equality constraint                      */
    mjCNSTR_EQUALITY         ,
    /** dof friction                             */
    mjCNSTR_FRICTION_DOF     ,
    /** tendon friction                          */
    mjCNSTR_FRICTION_TENDON  ,
    /** joint limit                              */
    mjCNSTR_LIMIT_JOINT      ,
    /** tendon limit                             */
    mjCNSTR_LIMIT_TENDON     ,
    /** frictionless contact                     */
    mjCNSTR_CONTACT_FRICTIONLESS,
    /** frictional contact, pyramidal friction cone */
    mjCNSTR_CONTACT_PYRAMIDAL,
    /** frictional contact, elliptic friction cone */
    mjCNSTR_CONTACT_ELLIPTIC ,
}
/**  constraint state                        */
export enum mjtConstraintState {
    /** constraint satisfied, zero cost (limit, contact) */
    mjCNSTRSTATE_SATISFIED   ,
    /** quadratic cost (equality, friction, limit, contact) */
    mjCNSTRSTATE_QUADRATIC   ,
    /** linear cost, negative side (friction)    */
    mjCNSTRSTATE_LINEARNEG   ,
    /** linear cost, positive side (friction)    */
    mjCNSTRSTATE_LINEARPOS   ,
    /** squared distance to cone cost (elliptic contact) */
    mjCNSTRSTATE_CONE        ,
}
/**  type of sensor                          */
export enum mjtSensor {
    /** scalar contact normal forces summed over sensor zone */
    mjSENS_TOUCH             ,
    /** 3D linear acceleration, in local frame   */
    mjSENS_ACCELEROMETER     ,
    /** 3D linear velocity, in local frame       */
    mjSENS_VELOCIMETER       ,
    /** 3D angular velocity, in local frame      */
    mjSENS_GYRO              ,
    /** 3D force between site's body and its parent body */
    mjSENS_FORCE             ,
    /** 3D torque between site's body and its parent body */
    mjSENS_TORQUE            ,
    /** 3D magnetometer                          */
    mjSENS_MAGNETOMETER      ,
    /** scalar distance to nearest geom or site along z-axis */
    mjSENS_RANGEFINDER       ,
    /** pixel coordinates of a site in the camera image */
    mjSENS_CAMPROJECTION     ,
    /** scalar joint position (hinge and slide only) */
    mjSENS_JOINTPOS          ,
    /** scalar joint velocity (hinge and slide only) */
    mjSENS_JOINTVEL          ,
    /** scalar tendon position                   */
    mjSENS_TENDONPOS         ,
    /** scalar tendon velocity                   */
    mjSENS_TENDONVEL         ,
    /** scalar actuator position                 */
    mjSENS_ACTUATORPOS       ,
    /** scalar actuator velocity                 */
    mjSENS_ACTUATORVEL       ,
    /** scalar actuator force                    */
    mjSENS_ACTUATORFRC       ,
    /** scalar actuator force, measured at the joint */
    mjSENS_JOINTACTFRC       ,
    /** scalar actuator force, measured at the tendon */
    mjSENS_TENDONACTFRC      ,
    /** 4D ball joint quaternion                 */
    mjSENS_BALLQUAT          ,
    /** 3D ball joint angular velocity           */
    mjSENS_BALLANGVEL        ,
    /** joint limit distance-margin              */
    mjSENS_JOINTLIMITPOS     ,
    /** joint limit velocity                     */
    mjSENS_JOINTLIMITVEL     ,
    /** joint limit force                        */
    mjSENS_JOINTLIMITFRC     ,
    /** tendon limit distance-margin             */
    mjSENS_TENDONLIMITPOS    ,
    /** tendon limit velocity                    */
    mjSENS_TENDONLIMITVEL    ,
    /** tendon limit force                       */
    mjSENS_TENDONLIMITFRC    ,
    /** 3D position                              */
    mjSENS_FRAMEPOS          ,
    /** 4D unit quaternion orientation           */
    mjSENS_FRAMEQUAT         ,
    /** 3D unit vector: x-axis of object's frame */
    mjSENS_FRAMEXAXIS        ,
    /** 3D unit vector: y-axis of object's frame */
    mjSENS_FRAMEYAXIS        ,
    /** 3D unit vector: z-axis of object's frame */
    mjSENS_FRAMEZAXIS        ,
    /** 3D linear velocity                       */
    mjSENS_FRAMELINVEL       ,
    /** 3D angular velocity                      */
    mjSENS_FRAMEANGVEL       ,
    /** 3D linear acceleration                   */
    mjSENS_FRAMELINACC       ,
    /** 3D angular acceleration                  */
    mjSENS_FRAMEANGACC       ,
    /** 3D center of mass of subtree             */
    mjSENS_SUBTREECOM        ,
    /** 3D linear velocity of subtree            */
    mjSENS_SUBTREELINVEL     ,
    /** 3D angular momentum of subtree           */
    mjSENS_SUBTREEANGMOM     ,
    /** signed distance between two geoms        */
    mjSENS_GEOMDIST          ,
    /** normal direction between two geoms       */
    mjSENS_GEOMNORMAL        ,
    /** segment between two geoms                */
    mjSENS_GEOMFROMTO        ,
    /** potential energy                         */
    mjSENS_E_POTENTIAL       ,
    /** kinetic energy                           */
    mjSENS_E_KINETIC         ,
    /** simulation time                          */
    mjSENS_CLOCK             ,
    /** plugin-controlled                        */
    mjSENS_PLUGIN            ,
    /** sensor data provided by mjcb_sensor callback */
    mjSENS_USER              ,
}
/**  computation stage                       */
export enum mjtStage {
    /** no computations                          */
    mjSTAGE_NONE             ,
    /** position-dependent computations          */
    mjSTAGE_POS              ,
    /** velocity-dependent computations          */
    mjSTAGE_VEL              ,
    /** acceleration/force-dependent computations */
    mjSTAGE_ACC              ,
}
/**  data type for sensors                   */
export enum mjtDataType {
    /** real values, no constraints              */
    mjDATATYPE_REAL          ,
    /** positive values; 0 or negative: inactive */
    mjDATATYPE_POSITIVE      ,
    /** 3D unit vector                           */
    mjDATATYPE_AXIS          ,
    /** unit quaternion                          */
    mjDATATYPE_QUATERNION    ,
}
/**  frame alignment of bodies with their children */
export enum mjtSameFrame {
    /** no alignment                             */
    mjSAMEFRAME_NONE         ,
    /** frame is same as body frame              */
    mjSAMEFRAME_BODY         ,
    /** frame is same as inertial frame          */
    mjSAMEFRAME_INERTIA      ,
    /** frame orientation is same as body orientation */
    mjSAMEFRAME_BODYROT      ,
    /** frame orientation is same as inertia orientation */
    mjSAMEFRAME_INERTIAROT   ,
}
/**  mode for actuator length range computation */
export enum mjtLRMode {
    /** do not process any actuators             */
    mjLRMODE_NONE            ,
    /** process muscle actuators                 */
    mjLRMODE_MUSCLE          ,
    /** process muscle and user actuators        */
    mjLRMODE_MUSCLEUSER      ,
    /** process all actuators                    */
    mjLRMODE_ALL             ,
}
/**  mode for flex selfcollide               */
export enum mjtFlexSelf {
    /** no self-collisions                       */
    mjFLEXSELF_NONE          ,
    /** skip midphase, go directly to narrowphase */
    mjFLEXSELF_NARROW        ,
    /** use BVH in midphase (if midphase enabled) */
    mjFLEXSELF_BVH           ,
    /** use SAP in midphase                      */
    mjFLEXSELF_SAP           ,
    /** choose between BVH and SAP automatically */
    mjFLEXSELF_AUTO          ,
}

export interface Model {
  new (filename : string) : Model;
  load_from_xml(str: string): Model;
  /** Free the memory associated with the model */
  free(): void;
  /** Retrive various parameters of the current simulation */
  getOptions(): any;
  // MODEL_INTERFACE

}

export interface State {
  new (model : Model) : State;
  /** Free the memory associated with the state */
  free(): void;
}

export interface Simulation {
  new (model : Model, state : State) : Simulation;
  state() : State;
  model() : Model;
  /** Free the memory associated with both the model and the state in the simulation */
  free()  : void;
  /** Apply cartesian force and torque (outside xfrc_applied mechanism) */
  applyForce(fx: number, fy: number, fz: number, tx: number, ty: number, tz: number, px: number, py: number, pz: number, body_id: number): void;
  
  /** sets perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise
   * d->qpos written only if flg_paused and subtree root for selected body has free joint */
  applyPose(bodyID: number,
            refPosX : number, refPosY : number, refPosZ : number,
            refQuat1: number, refQuat2: number, refQuat3: number, refQuat4: number,
            flg_paused: number): void;
  // DATA_INTERFACE
  /** Free last XML model if loaded. Called internally at each load.*/
  freeLastXML           (): void;
  /** Advance simulation, use control callback to obtain external force and control.*/
  step                  (): void;
  /** Advance simulation in two steps: before external force and control is set by user.*/
  step1                 (): void;
  /** Advance simulation in two steps: after external force and control is set by user.*/
  step2                 (): void;
  /** Forward dynamics: same as mj_step but do not integrate in time.*/
  forward               (): void;
  /** Inverse dynamics: qacc must be set before calling.*/
  inverse               (): void;
  /** Forward dynamics with skip; skipstage is mjtStage.*/
  forwardSkip           (skipstage : number, skipsensor : number): void;
  /** Inverse dynamics with skip; skipstage is mjtStage.*/
  inverseSkip           (skipstage : number, skipsensor : number): void;
  /** Set solver parameters to default values.    [Only works with MuJoCo Allocated Arrays!]*/
  defaultSolRefImp      (solref : Float64Array, solimp : Float64Array): void;
  /** Return size of buffer needed to hold model.*/
  sizeModel             (): number;
  /** Reset data to defaults.*/
  resetData             (): void;
  /** Reset data to defaults, fill everything else with debug_value.*/
  resetDataDebug        (debug_value : string): void;
  /** Reset data, set fields from specified keyframe.*/
  resetDataKeyframe     (key : number): void;
  /** Free memory allocation in mjData.*/
  deleteData            (): void;
  /** Reset all callbacks to NULL pointers (NULL is the default).*/
  resetCallbacks        (): void;
  /** Print mjModel to text file, specifying format. float_format must be a valid printf-style format string for a single float value.*/
  printFormattedModel   (filename : string, float_format : string): void;
  /** Print model to text file.*/
  printModel            (filename : string): void;
  /** Print mjData to text file, specifying format. float_format must be a valid printf-style format string for a single float value*/
  printFormattedData    (filename : string, float_format : string): void;
  /** Print data to text file.*/
  printData             (filename : string): void;
  /** Print matrix to screen.    [Only works with MuJoCo Allocated Arrays!]*/
  _printMat             (mat : Float64Array, nr : number, nc : number): void;
  /** Run position-dependent computations.*/
  fwdPosition           (): void;
  /** Run velocity-dependent computations.*/
  fwdVelocity           (): void;
  /** Compute actuator force qfrc_actuator.*/
  fwdActuation          (): void;
  /** Add up all non-constraint forces, compute qacc_smooth.*/
  fwdAcceleration       (): void;
  /** Run selected constraint solver.*/
  fwdConstraint         (): void;
  /** Euler integrator, semi-implicit in velocity.*/
  Euler                 (): void;
  /** Runge-Kutta explicit order-N integrator.*/
  RungeKutta            (N : number): void;
  /** Run position-dependent computations in inverse dynamics.*/
  invPosition           (): void;
  /** Run velocity-dependent computations in inverse dynamics.*/
  invVelocity           (): void;
  /** Apply the analytical formula for inverse constraint dynamics.*/
  invConstraint         (): void;
  /** Compare forward and inverse dynamics, save results in fwdinv.*/
  compareFwdInv         (): void;
  /** Evaluate position-dependent sensors.*/
  sensorPos             (): void;
  /** Evaluate velocity-dependent sensors.*/
  sensorVel             (): void;
  /** Evaluate acceleration and force-dependent sensors.*/
  sensorAcc             (): void;
  /** Evaluate position-dependent energy (potential).*/
  energyPos             (): void;
  /** Evaluate velocity-dependent energy (kinetic).*/
  energyVel             (): void;
  /** Check qpos, reset if any element is too big or nan.*/
  checkPos              (): void;
  /** Check qvel, reset if any element is too big or nan.*/
  checkVel              (): void;
  /** Check qacc, reset if any element is too big or nan.*/
  checkAcc              (): void;
  /** Run forward kinematics.*/
  kinematics            (): void;
  /** Map inertias and motion dofs to global frame centered at CoM.*/
  comPos                (): void;
  /** Compute camera and light positions and orientations.*/
  camlight              (): void;
  /** Compute tendon lengths, velocities and moment arms.*/
  tendon                (): void;
  /** Compute actuator transmission lengths and moments.*/
  transmission          (): void;
  /** Run composite rigid body inertia algorithm (CRB).*/
  crbCalculate          (): void;
  /** Compute sparse L'*D*L factorizaton of inertia matrix.*/
  factorM               (): void;
  /** Solve linear system M * x = y using factorization:  x = inv(L'*D*L)*y    [Only works with MuJoCo Allocated Arrays!]*/
  solveM                (x : Float64Array, y : Float64Array, n : number): void;
  /** Half of linear solve:  x = sqrt(inv(D))*inv(L')*y    [Only works with MuJoCo Allocated Arrays!]*/
  solveM2               (x : Float64Array, y : Float64Array, sqrtInvD : Float64Array, n : number): void;
  /** Compute cvel, cdof_dot.*/
  comVel                (): void;
  /** Compute qfrc_passive from spring-dampers, viscosity and density.*/
  passive               (): void;
  /** subtree linear velocity and angular momentum*/
  subtreeVel            (): void;
  /** RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term.    [Only works with MuJoCo Allocated Arrays!]*/
  rne                   (flg_acc : number, result : Float64Array): void;
  /** RNE with complete data: compute cacc, cfrc_ext, cfrc_int.*/
  rnePostConstraint     (): void;
  /** Run collision detection.*/
  collision             (): void;
  /** Construct constraints.*/
  makeConstraint        (): void;
  /** Compute inverse constraint inertia efc_AR.*/
  projectConstraint     (): void;
  /** Compute efc_vel, efc_aref.*/
  referenceConstraint   (): void;
  /** Determine type of friction cone.*/
  isPyramidal           (): number;
  /** Determine type of constraint Jacobian.*/
  isSparse              (): number;
  /** Determine type of solver (PGS is dual, CG and Newton are primal).*/
  isDual                (): number;
  /** Multiply dense or sparse constraint Jacobian by vector.    [Only works with MuJoCo Allocated Arrays!]*/
  mulJacVec             (res : Float64Array, vec : Float64Array): void;
  /** Multiply dense or sparse constraint Jacobian transpose by vector.    [Only works with MuJoCo Allocated Arrays!]*/
  mulJacTVec            (res : Float64Array, vec : Float64Array): void;
  /** Compute subtree center-of-mass end-effector Jacobian.    [Only works with MuJoCo Allocated Arrays!]*/
  jacSubtreeCom         (jacp : Float64Array, body : number): void;
  /** Get id of object with the specified mjtObj type and name, returns -1 if id not found.*/
  name2id               (type : number, name : string): number;
  /** Get name of object with the specified mjtObj type and id, returns NULL if name not found.*/
  id2name               (type : number, id : number): string;
  /** Convert sparse inertia matrix M into full (i.e. dense) matrix.    [Only works with MuJoCo Allocated Arrays!]*/
  fullM                 (dst : Float64Array, M : Float64Array): void;
  /** Compute velocity by finite-differencing two positions.    [Only works with MuJoCo Allocated Arrays!]*/
  differentiatePos      (qvel : Float64Array, dt : number, qpos1 : Float64Array, qpos2 : Float64Array): void;
  /** Integrate position with given velocity.    [Only works with MuJoCo Allocated Arrays!]*/
  integratePos          (qpos : Float64Array, qvel : Float64Array, dt : number): void;
  /** Normalize all quaternions in qpos-type vector.    [Only works with MuJoCo Allocated Arrays!]*/
  normalizeQuat         (qpos : Float64Array): void;
  /** Sum all body masses.*/
  getTotalmass          (): number;
  /** Return a config attribute value of a plugin instance; NULL: invalid plugin instance ID or attribute name*/
  getPluginConfig       (plugin_id : number, attrib : string): string;
  /** Load a dynamic library. The dynamic library is assumed to register one or more plugins.*/
  loadPluginLibrary     (path : string): void;
  /** Return version number: 1.0.2 is encoded as 102.*/
  version               (): number;
  /** Return the current version of MuJoCo as a null-terminated string.*/
  versionString         (): string;
  /** Draw rectangle.*/
  _rectangle            (viewport : mjrRect, r : number, g : number, b : number, a : number): void;
  /** Call glFinish.*/
  _finish               (): void;
  /** Call glGetError and return result.*/
  _getError             (): number;
  /** Get builtin UI theme spacing (ind: 0-1).*/
  i_themeSpacing        (ind : number): mjuiThemeSpacing;
  /** Get builtin UI theme color (ind: 0-3).*/
  i_themeColor          (ind : number): mjuiThemeColor;
  /** Main error function; does not return to caller.*/
  _error                (msg : string): void;
  /** Deprecated: use mju_error.*/
  _error_i              (msg : string, i : number): void;
  /** Deprecated: use mju_error.*/
  _error_s              (msg : string, text : string): void;
  /** Main warning function; returns to caller.*/
  _warning              (msg : string): void;
  /** Deprecated: use mju_warning.*/
  _warning_i            (msg : string, i : number): void;
  /** Deprecated: use mju_warning.*/
  _warning_s            (msg : string, text : string): void;
  /** Clear user error and memory handlers.*/
  _clearHandlers        (): void;
  /** High-level warning function: count warnings in mjData, print only the first.*/
  warning               (warning : number, info : number): void;
  /** Write [datetime, type: message] to MUJOCO_LOG.TXT.*/
  _writeLog             (type : string, msg : string): void;
  /** Set res = 0.    [Only works with MuJoCo Allocated Arrays!]*/
  _zero                 (res : Float64Array, n : number): void;
  /** Set res = val.    [Only works with MuJoCo Allocated Arrays!]*/
  _fill                 (res : Float64Array, val : number, n : number): void;
  /** Set res = vec.    [Only works with MuJoCo Allocated Arrays!]*/
  _copy                 (res : Float64Array, data : Float64Array, n : number): void;
  /** Return sum(vec).    [Only works with MuJoCo Allocated Arrays!]*/
  _sum                  (vec : Float64Array, n : number): number;
  /** Return L1 norm: sum(abs(vec)).    [Only works with MuJoCo Allocated Arrays!]*/
  _L1                   (vec : Float64Array, n : number): number;
  /** Set res = vec*scl.    [Only works with MuJoCo Allocated Arrays!]*/
  _scl                  (res : Float64Array, vec : Float64Array, scl : number, n : number): void;
  /** Set res = vec1 + vec2.    [Only works with MuJoCo Allocated Arrays!]*/
  _add                  (res : Float64Array, vec1 : Float64Array, vec2 : Float64Array, n : number): void;
  /** Set res = vec1 - vec2.    [Only works with MuJoCo Allocated Arrays!]*/
  _sub                  (res : Float64Array, vec1 : Float64Array, vec2 : Float64Array, n : number): void;
  /** Set res = res + vec.    [Only works with MuJoCo Allocated Arrays!]*/
  _addTo                (res : Float64Array, vec : Float64Array, n : number): void;
  /** Set res = res - vec.    [Only works with MuJoCo Allocated Arrays!]*/
  _subFrom              (res : Float64Array, vec : Float64Array, n : number): void;
  /** Set res = res + vec*scl.    [Only works with MuJoCo Allocated Arrays!]*/
  _addToScl             (res : Float64Array, vec : Float64Array, scl : number, n : number): void;
  /** Set res = vec1 + vec2*scl.    [Only works with MuJoCo Allocated Arrays!]*/
  _addScl               (res : Float64Array, vec1 : Float64Array, vec2 : Float64Array, scl : number, n : number): void;
  /** Normalize vector, return length before normalization.    [Only works with MuJoCo Allocated Arrays!]*/
  _normalize            (res : Float64Array, n : number): number;
  /** Return vector length (without normalizing vector).    [Only works with MuJoCo Allocated Arrays!]*/
  _norm                 (res : Float64Array, n : number): number;
  /** Return dot-product of vec1 and vec2.    [Only works with MuJoCo Allocated Arrays!]*/
  _dot                  (vec1 : Float64Array, vec2 : Float64Array, n : number): number;
  /** Multiply matrix and vector: res = mat * vec.    [Only works with MuJoCo Allocated Arrays!]*/
  _mulMatVec            (res : Float64Array, mat : Float64Array, vec : Float64Array, nr : number, nc : number): void;
  /** Multiply transposed matrix and vector: res = mat' * vec.    [Only works with MuJoCo Allocated Arrays!]*/
  _mulMatTVec           (res : Float64Array, mat : Float64Array, vec : Float64Array, nr : number, nc : number): void;
  /** Multiply square matrix with vectors on both sides: returns vec1' * mat * vec2.    [Only works with MuJoCo Allocated Arrays!]*/
  _mulVecMatVec         (vec1 : Float64Array, mat : Float64Array, vec2 : Float64Array, n : number): number;
  /** Transpose matrix: res = mat'.    [Only works with MuJoCo Allocated Arrays!]*/
  _transpose            (res : Float64Array, mat : Float64Array, nr : number, nc : number): void;
  /** Symmetrize square matrix res = (mat + mat')/2.    [Only works with MuJoCo Allocated Arrays!]*/
  _symmetrize           (res : Float64Array, mat : Float64Array, n : number): void;
  /** Set mat to the identity matrix.    [Only works with MuJoCo Allocated Arrays!]*/
  _eye                  (mat : Float64Array, n : number): void;
  /** Multiply matrices: res = mat1 * mat2.    [Only works with MuJoCo Allocated Arrays!]*/
  _mulMatMat            (res : Float64Array, mat1 : Float64Array, mat2 : Float64Array, r1 : number, c1 : number, c2 : number): void;
  /** Multiply matrices, second argument transposed: res = mat1 * mat2'.    [Only works with MuJoCo Allocated Arrays!]*/
  _mulMatMatT           (res : Float64Array, mat1 : Float64Array, mat2 : Float64Array, r1 : number, c1 : number, r2 : number): void;
  /** Multiply matrices, first argument transposed: res = mat1' * mat2.    [Only works with MuJoCo Allocated Arrays!]*/
  _mulMatTMat           (res : Float64Array, mat1 : Float64Array, mat2 : Float64Array, r1 : number, c1 : number, c2 : number): void;
  /** Set res = mat' * diag * mat if diag is not NULL, and res = mat' * mat otherwise.    [Only works with MuJoCo Allocated Arrays!]*/
  _sqrMatTD             (res : Float64Array, mat : Float64Array, diag : Float64Array, nr : number, nc : number): void;
  /** Cholesky decomposition: mat = L*L'; return rank, decomposition performed in-place into mat.    [Only works with MuJoCo Allocated Arrays!]*/
  _cholFactor           (mat : Float64Array, n : number, mindiag : number): number;
  /** Solve mat * res = vec, where mat is Cholesky-factorized    [Only works with MuJoCo Allocated Arrays!]*/
  _cholSolve            (res : Float64Array, mat : Float64Array, vec : Float64Array, n : number): void;
  /** Cholesky rank-one update: L*L' +/- x*x'; return rank.    [Only works with MuJoCo Allocated Arrays!]*/
  _cholUpdate           (mat : Float64Array, x : Float64Array, n : number, flg_plus : number): number;
  /** Convert contact force to pyramid representation.    [Only works with MuJoCo Allocated Arrays!]*/
  _encodePyramid        (pyramid : Float64Array, force : Float64Array, mu : Float64Array, dim : number): void;
  /** Convert pyramid representation to contact force.    [Only works with MuJoCo Allocated Arrays!]*/
  _decodePyramid        (force : Float64Array, pyramid : Float64Array, mu : Float64Array, dim : number): void;
  /** Integrate spring-damper analytically, return pos(dt).*/
  _springDamper         (pos0 : number, vel0 : number, Kp : number, Kv : number, dt : number): number;
  /** Return min(a,b) with single evaluation of a and b.*/
  _min                  (a : number, b : number): number;
  /** Return max(a,b) with single evaluation of a and b.*/
  _max                  (a : number, b : number): number;
  /** Clip x to the range [min, max].*/
  _clip                 (x : number, min : number, max : number): number;
  /** Return sign of x: +1, -1 or 0.*/
  _sign                 (x : number): number;
  /** Round x to nearest integer.*/
  _round                (x : number): number;
  /** Convert type id (mjtObj) to type name.*/
  _type2Str             (type : number): string;
  /** Convert type name to type id (mjtObj).*/
  _str2Type             (str : string): number;
  /** Return human readable number of bytes using standard letter suffix.*/
  _writeNumBytes        (nbytes : number): string;
  /** Construct a warning message given the warning type and info.*/
  _warningText          (warning : number, info : number): string;
  /** Return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise. Used by check functions.*/
  _isBad                (x : number): number;
  /** Return 1 if all elements are 0.    [Only works with MuJoCo Allocated Arrays!]*/
  _isZero               (vec : Float64Array, n : number): number;
  /** Standard normal random number generator (optional second number).    [Only works with MuJoCo Allocated Arrays!]*/
  _standardNormal       (num2 : Float64Array): number;
  /** Insertion sort, resulting list is in increasing order.    [Only works with MuJoCo Allocated Arrays!]*/
  _insertionSort        (list : Float64Array, n : number): void;
  /** Generate Halton sequence.*/
  _Halton               (index : number, base : number): number;
  /** Sigmoid function over 0<=x<=1 constructed from half-quadratics.*/
  _sigmoid              (x : number): number;
  /** Finite differenced transition matrices (control theory notation)   d(x_next) = A*dx + B*du   d(sensor) = C*dx + D*du   required output matrix dimensions:      A: (2*nv+na x 2*nv+na)      B: (2*nv+na x nu)      D: (nsensordata x 2*nv+na)      C: (nsensordata x nu)    [Only works with MuJoCo Allocated Arrays!]*/
  _transitionFD         (eps : number, centered : mjtByte, A : Float64Array, B : Float64Array, C : Float64Array, D : Float64Array): void;
  /** Return the number of globally registered plugins.*/
  _pluginCount          (): number;
}

export interface mujoco extends EmscriptenModule {
  FS    : typeof FS;
  MEMFS : typeof MEMFS;
  Model : Model;
  State : State;
  Simulation : Simulation;
}
declare var load_mujoco: EmscriptenModuleFactory<mujoco>;
export default load_mujoco;
