(ns pollen-robotics.main
  "Kotodama WASM entrypoint for the Pollen Robotics (Reachy 2) clean-room
  actor (L5).

  Clean-room, API-compatible surface for the Reachy 2 developer SDK. Derived
  from the published resource shapes in pollen-robotics/reachy2-sdk-api
  (protos/, Apache-2.0) and docs.pollen-robotics.com; no upstream code,
  generated stub or credential is reproduced.

  Three surfaces, one decision:

    schema/pollen_robotics.kotoba   the resource shapes, EAVT-mapped
    cores/*.cljk                    the decisions, in Kotoba
    src/pollen_robotics/main.cljc   this: the host, and the cores' oracle

  Everything under `--- decision oracle ---` is a line-for-line restatement of
  the two `.cljk` cores. It exists so the host can answer without a WASM
  round trip AND so `main_test.cljc` can pin the two against each other: if
  the Kotoba core and this file ever disagree, the parity corpus fails rather
  than one of them silently winning. The cores are authoritative; this is the
  copy.

  Units: the SDK presents degrees, metres and seconds as floats. Decisions are
  made in integer millidegrees, millimetres and milliseconds, converted once
  here at the boundary, so a limit check that decides whether a motor moves
  does not depend on which backend rounded the compare.

  State lives on the kotoba Datom log: `emit-facts` produces namespaced EAVT
  facts (`pollen_robotics.<Entity>/<field>`); `*store*` is the in-memory
  materialization used by the contract test and by the WASM runtime before a
  live engine binds. This actor is policy and bookkeeping, not control -- it
  never drives a motor; it produces the record a governor
  (kotoba-lang/robotics) needs to refuse unsafe actuation before actuation."
  (:require [kotoba.lang.text :as str]))

(def ns-prefix "pollen_robotics")
(def tier "L5")
(def default-limit 20)
(def max-limit 100)

;; --- schema-derived entity specs (the single source the handlers fold over) ---
(def entity-specs
  [{:entity "ReachyInfo" :plural "reachyinfos" :id-prefix "reachy_info"
    :fields [:name :serial_number :version_hard :version_soft]
    :required [:name :serial_number]
    :coerce {}
    :refs {}}
   {:entity "Part" :plural "parts" :id-prefix "reachy_part"
    :fields [:name :kind :activated :serial_number :version_hard :version_soft]
    :required [:name :kind]
    :coerce {:activated :bool}
    :refs {}}
   {:entity "ArmJointState" :plural "armjointstates" :id-prefix "reachy_ajs"
    :fields [:part :shoulder_pitch :shoulder_roll :elbow_yaw :elbow_pitch
             :wrist_roll :wrist_pitch :wrist_yaw]
    :required [:part]
    :coerce {:shoulder_pitch :float :shoulder_roll :float :elbow_yaw :float
             :elbow_pitch :float :wrist_roll :float :wrist_pitch :float
             :wrist_yaw :float}
    :refs {:part "Part"}}
   {:entity "JointLimit" :plural "jointlimits" :id-prefix "reachy_lim"
    :fields [:part :joint :min :max]
    :required [:part :joint :min :max]
    :coerce {:min :float :max :float}
    :refs {:part "Part"}}
   {:entity "EndEffectorPose" :plural "endeffectorposes" :id-prefix "reachy_eef"
    :fields [:part :tx :ty :tz :qw :qx :qy :qz]
    :required [:part]
    :coerce {:tx :float :ty :float :tz :float
             :qw :float :qx :float :qy :float :qz :float}
    :refs {:part "Part"}}
   {:entity "NeckOrientation" :plural "neckorientations" :id-prefix "reachy_neck"
    :fields [:part :roll :pitch :yaw]
    :required [:part]
    :coerce {:roll :float :pitch :float :yaw :float}
    :refs {:part "Part"}}
   {:entity "AntennaState" :plural "antennastates" :id-prefix "reachy_ant"
    :fields [:part :joint :present_position :goal_position :temperature :compliant]
    :required [:part :joint]
    :coerce {:present_position :float :goal_position :float
             :temperature :float :compliant :bool}
    :refs {:part "Part"}}
   {:entity "HandState" :plural "handstates" :id-prefix "reachy_hand"
    :fields [:part :opening :force :holding_object :compliant
             :temperature_motor :temperature_driver]
    :required [:part]
    :coerce {:opening :float :force :float :holding_object :bool :compliant :bool
             :temperature_motor :float :temperature_driver :float}
    :refs {:part "Part"}}
   {:entity "GoToRequest" :plural "gotos" :id-prefix "reachy_goto"
    :fields [:part :space :interpolation_mode :duration :status :order_id]
    :required [:part :space :interpolation_mode :duration]
    :coerce {:interpolation_mode :int :duration :float :status :int :order_id :int}
    :refs {:part "Part"}}
   {:entity "MobileBaseState" :plural "mobilebasestates" :id-prefix "reachy_base"
    :fields [:part :x :y :theta :battery_level :zuuu_mode :control_mode]
    :required [:part]
    :coerce {:x :float :y :float :theta :float :battery_level :float
             :zuuu_mode :int :control_mode :int}
    :refs {:part "Part"}}
   {:entity "LidarSafety" :plural "lidarsafeties" :id-prefix "reachy_lidar"
    :fields [:part :safety_on :safety_distance :critical_distance
             :obstacle_distance :obstacle_detection_status]
    :required [:part]
    :coerce {:safety_on :bool :safety_distance :float :critical_distance :float
             :obstacle_distance :float :obstacle_detection_status :int}
    :refs {:part "Part"}}
   {:entity "CameraFeature" :plural "camerafeatures" :id-prefix "reachy_cam"
    :fields [:name :stereo :depth :height :width :distortion_model]
    :required [:name]
    :coerce {:stereo :bool :depth :bool :height :int :width :int}
    :refs {}}])

(def entities (mapv :entity entity-specs))

;; --- the parts a Reachy 2 exposes (reachy.proto: Reachy) ---
(def part-names ["l_arm" "r_arm" "head" "l_hand" "r_hand" "mobile_base"])

;; ArmJoints / NeckJoints, in the order the protos declare them. The Python
;; SDK's goto() list is this order, and the order is part of the contract.
(def arm-joints
  ["shoulder.pitch" "shoulder.roll" "elbow.yaw" "elbow.pitch"
   "wrist.roll" "wrist.pitch" "wrist.yaw"])
(def neck-joints ["neck.roll" "neck.pitch" "neck.yaw"])

(defn joints-for-part
  "Which joint list a goto against this part must supply. nil for a part that
  takes no joint-space goto (hands take an opening, the base takes a pose)."
  [part]
  (cond (contains? #{"l_arm" "r_arm"} part) arm-joints
        (= part "head") neck-joints
        :else nil))

;; --- routes ---
(def crud-routes
  (vec (mapcat (fn [{:keys [plural entity]}]
                 [{:method "POST"   :path (str "/v1/" plural)        :op (str "create " entity) :entity entity}
                  {:method "GET"    :path (str "/v1/" plural)        :op (str "list " entity)   :entity entity}
                  {:method "GET"    :path (str "/v1/" plural "/{id}") :op (str "get " entity)    :entity entity}
                  {:method "PATCH"  :path (str "/v1/" plural "/{id}") :op (str "update " entity) :entity entity}
                  {:method "DELETE" :path (str "/v1/" plural "/{id}") :op (str "delete " entity) :entity entity}])
               entity-specs)))

;; The SDK's commands are not CRUD, and pretending they are would lose the
;; only thing that makes them interesting: they are refusable. Each of these
;; routes runs a decision core and can answer "no".
(def command-routes
  [{:method "POST" :path "/v1/gotos/admit"          :op "admit GoToRequest"        :entity "GoToRequest"}
   {:method "POST" :path "/v1/gotos/{id}/cancel"    :op "cancel GoToRequest"       :entity "GoToRequest"}
   {:method "POST" :path "/v1/gotos/{id}/status"    :op "advance GoToRequest"      :entity "GoToRequest"}
   {:method "GET"  :path "/v1/lidarsafeties/{id}/gate" :op "gate MobileBaseState"  :entity "LidarSafety"}])

(def routes (into crud-routes command-routes))

;; --- platform primitives ---
(defn now []
  #?(:clj (str (java.time.Instant/now))
     :cljs (.toISOString (js/Date.))))

(defn- rand-hex16 []
  #?(:clj (subs (str/replace (str (java.util.UUID/randomUUID)) "-" "") 0 16)
     :cljs (subs (str/replace (str (random-uuid)) "-" "") 0 16)))

(defn new-id [prefix] (str prefix "_" (rand-hex16)))

;; --- coercion ---
(defn as-int [v]
  (cond (number? v) (long v)
        (string? v) (try #?(:clj (Long/parseLong (str/trim v))
                            :cljs (let [n (js/parseInt v 10)] (if (js/isNaN n) 0 n)))
                         (catch #?(:clj Exception :cljs :default) _ 0))
        :else 0))

(defn as-float [v]
  (cond (number? v) (double v)
        (string? v) (try #?(:clj (Double/parseDouble (str/trim v))
                            :cljs (let [n (js/parseFloat v)] (if (js/isNaN n) 0.0 n)))
                         (catch #?(:clj Exception :cljs :default) _ 0.0))
        :else 0.0))

(defn as-bool [v]
  (if (nil? v) false (contains? #{"1" "true" "yes" "on" true} (if (string? v) (str/lower v) v))))

(defn coerce-field [kind v]
  (case kind :int (as-int v) :float (as-float v) :bool (as-bool v) v))

;; --- host boundary: float SI -> integer fixed point ---
;;
;; The one place a float becomes a decision input. Half-up on both hosts
;; (Math.round / js Math.round agree), so a value on the boundary lands the
;; same way whichever backend evaluates it.
(defn- round-long [x]
  #?(:clj (Math/round (double x)) :cljs (long (js/Math.round x))))

(defn deg->millideg [deg] (round-long (* (as-float deg) 1000)))
(defn m->mm [m] (round-long (* (as-float m) 1000)))
(defn s->ms [s] (round-long (* (as-float s) 1000)))

;; =========================================================================
;; --- decision oracle -----------------------------------------------------
;;
;; Restatement of cores/reachy_lidar_safety_core.cljk and
;; cores/reachy_goto_core.cljk. Same names, same order, same fail-closed
;; branches. The cores are authoritative; `main_test.cljc` pins this copy to
;; them over a shared corpus.
;; =========================================================================

;; -- LidarObstacleDetectionEnum (mobile_base_lidar.proto) --
(def status-detection-error 0)
(def status-no-object 1)
(def status-slowdown 2)
(def status-stop 3)

(def scale-full 1000)
(def scale-stop 0)

(defn reading-valid? [distance-mm] (>= distance-mm 0))

(defn distances-sane? [safety-mm critical-mm]
  (if (<= critical-mm 0) false (< critical-mm safety-mm)))

(defn obstacle-status [distance-mm safety-mm critical-mm]
  (if-not (reading-valid? distance-mm)
    status-detection-error
    (if-not (distances-sane? safety-mm critical-mm)
      status-detection-error
      (cond (<= distance-mm critical-mm) status-stop
            (<= distance-mm safety-mm)   status-slowdown
            :else                        status-no-object))))

(defn ramp-permille [distance-mm safety-mm critical-mm]
  (let [span (- safety-mm critical-mm)
        over (- distance-mm critical-mm)]
    (if (or (<= span 0) (<= over 0))
      scale-stop
      (let [raw (quot (* over 1000) span)]
        (if (> raw scale-full) scale-full raw)))))

(defn sdk-speed-scale-permille
  "What the robot does: LidarSafety.safety_on is an operator switch, and with
  it off the base executes the commanded speed."
  [safety-on distance-mm safety-mm critical-mm]
  (if-not safety-on
    scale-full
    (let [s (obstacle-status distance-mm safety-mm critical-mm)]
      (cond (= s status-no-object) scale-full
            (= s status-slowdown)  (ramp-permille distance-mm safety-mm critical-mm)
            :else                  scale-stop))))

(defn governed-speed-scale-permille
  "What an actor behind a governor may command: the operator switch may relax
  a SLOWDOWN, but it may not turn a STOP -- or an unreadable lidar -- into
  motion. Deliberately not equal to `sdk-speed-scale-permille`; see the core."
  [safety-on distance-mm safety-mm critical-mm]
  (let [s (obstacle-status distance-mm safety-mm critical-mm)]
    (cond (= s status-stop)            scale-stop
          (= s status-detection-error) scale-stop
          (not safety-on)              scale-full
          (= s status-no-object)       scale-full
          :else (ramp-permille distance-mm safety-mm critical-mm))))

(defn motion-admitted? [safety-on distance-mm safety-mm critical-mm]
  (> (governed-speed-scale-permille safety-on distance-mm safety-mm critical-mm) 0))

;; -- InterpolationMode / GoalStatus (goto.proto) --
(def interpolation-none 0)
(def interpolation-linear 1)
(def interpolation-minimum-jerk 2)

(def goal-status-none 0)
(def goal-status-unknown 1)
(def goal-status-accepted 2)
(def goal-status-executing 3)
(def goal-status-canceling 4)
(def goal-status-succeeded 5)
(def goal-status-canceled 6)
(def goal-status-aborted 7)

(def arm-joint-count 7)
(def neck-joint-count 3)
(def min-duration-ms 1)
(def max-duration-ms 600000)

(def reason-ok "ok")
(def reason-part-not-ready "part-not-ready")
(def reason-joint-count "joint-count-mismatch")
(def reason-interpolation "interpolation-mode-not-admitted")
(def reason-duration "duration-out-of-range")
(def reason-joint-limit "joint-limit-exceeded")

(defn interpolation-admitted? [mode]
  (or (= mode interpolation-linear) (= mode interpolation-minimum-jerk)))

(defn duration-admitted? [duration-ms]
  (if (< duration-ms min-duration-ms) false (<= duration-ms max-duration-ms)))

(defn joint-count-ok? [n expected] (= n expected))

(defn within-limit? [q-millideg min-millideg max-millideg]
  (if (> min-millideg max-millideg)
    false
    (if (< q-millideg min-millideg) false (<= q-millideg max-millideg))))

(defn goto-shape-reason [part-ready joint-count expected-count interpolation duration-ms]
  (cond (not part-ready) reason-part-not-ready
        (not (joint-count-ok? joint-count expected-count)) reason-joint-count
        (not (interpolation-admitted? interpolation)) reason-interpolation
        (not (duration-admitted? duration-ms)) reason-duration
        :else reason-ok))

(defn admit-reason [shape-reason limits-ok]
  (if-not (= shape-reason reason-ok)
    shape-reason
    (if-not limits-ok reason-joint-limit reason-ok)))

(defn admitted? [shape-reason limits-ok] (= (admit-reason shape-reason limits-ok) reason-ok))

(defn admit-code [shape-reason limits-ok]
  (let [r (admit-reason shape-reason limits-ok)]
    (cond (= r reason-ok) 0
          (= r reason-part-not-ready) 1
          (= r reason-joint-count) 2
          (= r reason-interpolation) 3
          (= r reason-duration) 4
          :else 5)))

(defn terminal? [status]
  (contains? #{goal-status-succeeded goal-status-canceled goal-status-aborted} status))

(defn active? [status]
  (contains? #{goal-status-accepted goal-status-executing goal-status-canceling} status))

(defn cancellable? [status]
  (contains? #{goal-status-accepted goal-status-executing} status))

(defn status-after-cancel [status]
  (if (cancellable? status) goal-status-canceling status))

(defn transition-admitted? [from to]
  (cond (terminal? from) false
        (= from goal-status-accepted)
        (contains? #{goal-status-executing goal-status-canceling goal-status-aborted} to)
        (= from goal-status-executing)
        (contains? #{goal-status-succeeded goal-status-canceling goal-status-aborted} to)
        (= from goal-status-canceling)
        (contains? #{goal-status-canceled goal-status-aborted} to)
        :else false))

;; =========================================================================
;; --- store (materializes the Datom log; live engine binds in prod) -------
;; =========================================================================

(defn fresh-store [] (atom {}))
(def ^:dynamic *store* (fresh-store))

(defn emit-facts
  "EAVT facts for one record: {\"pollen_robotics.<Entity>/<field>\" v ...}. The
  datomic binding transacts these; the in-memory store keeps the record by id."
  [entity rec]
  (into {} (map (fn [[k v]] [(str ns-prefix "." entity "/" (name k)) v]) rec)))

(defn persist! [store entity rec]
  (swap! store assoc-in [entity (:id rec)] rec)
  rec)

(defn query
  ([store entity] (vec (vals (get @store entity))))
  ([store entity id] (if-let [r (get-in @store [entity id])] [r] [])))

(defn retract! [store entity id] (swap! store update entity dissoc id) {:id id :deleted true})

;; --- validation ---
(defn require-fields [data fields]
  (let [missing (remove #(let [v (get data %)] (and (some? v) (not= v ""))) fields)]
    (when (seq missing)
      {:error {:message (str "Missing required fields: " (str/join ", " (map name missing)))
               :type "invalid_request_error"}})))

(defn reject-unknown [data allowed]
  (let [allowed-set (set allowed)
        extra (remove allowed-set (keys data))]
    (when (seq extra)
      {:error {:message (str "Unknown fields: " (str/join ", " (map name extra)))
               :type "invalid_request_error"}})))

;; --- list helpers ---
(defn apply-filters [rows params fields]
  (reduce (fn [out f]
            (let [want (get params f)]
              (if (and (some? want) (not= want ""))
                (filterv #(= (str (get % f)) (str want)) out)
                out)))
          rows fields))

(defn- index-of-id [rows id]
  (first (keep-indexed (fn [i r] (when (= (:id r) id) i)) rows)))

(defn paginate [rows params]
  (let [asked (as-int (get params :limit))
        limit (min (max (if (pos? asked) asked default-limit) 1) max-limit)
        start (get params :starting_after)
        rows (if-let [i (and (some? start) (index-of-id rows start))]
               (vec (drop (inc i) rows))
               rows)
        page (vec (take limit rows))]
    [page (> (count rows) limit)]))

(defn expand [store rec params refs]
  (let [want (set (str/split (or (get params :expand) "") #","))]
    (reduce (fn [r [field ent]]
              (if (and (contains? want (name field)) (get r field))
                (assoc r (keyword (str (name field) "_obj")) (first (query store ent (get r field))))
                r))
            rec refs)))

;; --- generic handlers (return [body status]) ---
(defn- spec-for [entity] (first (filter #(= (:entity %) entity) entity-specs)))
(defn- not-found [] [{:error {:message "Not found" :type "not_found"}} 404])

(defn handle-create [store entity data]
  (let [{:keys [fields required coerce id-prefix]} (spec-for entity)]
    (or (some-> (reject-unknown data fields) (vector 400))
        (some-> (require-fields data required) (vector 400))
        (let [base {:id (new-id id-prefix)}
              rec (reduce (fn [m f] (assoc m f (coerce-field (get coerce f) (get data f)))) base fields)
              rec (assoc rec :createdAt (now) :updatedAt (now))]
          (persist! store entity rec)
          [rec 201]))))

(defn handle-list [store entity params]
  (let [{:keys [fields]} (spec-for entity)
        rows (apply-filters (query store entity) params fields)
        [page has-more] (paginate rows params)]
    [{:object "list" :data page :has_more has-more :count (count page) :total (count rows)} 200]))

(defn handle-get [store entity id params]
  (let [{:keys [refs]} (spec-for entity) rows (query store entity id)]
    (if (empty? rows) (not-found) [(expand store (first rows) params refs) 200])))

(defn handle-update [store entity id data]
  (let [{:keys [fields]} (spec-for entity) rows (query store entity id)]
    (if (empty? rows)
      (not-found)
      (or (some-> (reject-unknown data fields) (vector 400))
          (let [rec (reduce-kv (fn [m k v] (if (#{:id :createdAt} k) m (assoc m k v)))
                               (first rows) data)
                rec (assoc rec :updatedAt (now))]
            (persist! store entity rec)
            [rec 200])))))

(defn handle-delete [store entity id]
  (if (empty? (query store entity id)) (not-found) [(retract! store entity id) 200]))

;; =========================================================================
;; --- command handlers: the refusable surface -----------------------------
;; =========================================================================

(defn part-ready?
  "A part is ready when the robot reported it AND reported it activated.
  A part this actor has never seen is not ready -- absence of a record is not
  evidence of a compliant motor."
  [store part]
  (boolean (some (fn [p] (and (= (:name p) part) (true? (:activated p))))
                 (query store "Part"))))

(defn limits-for
  "The limit table the ROBOT reported for one part, as {joint [min max]} in
  millidegrees. A joint the robot never reported is absent, and the caller
  treats absence as a refusal -- it does not substitute a default."
  [store part]
  (into {} (for [l (query store "JointLimit") :when (= (:part l) part)]
             [(:joint l) [(deg->millideg (:min l)) (deg->millideg (:max l))]])))

(defn joint-limits-ok?
  "Fold `within-limit?` over the reported table. `targets-deg` is positional,
  in the order `joints-for-part` declares. A missing limit pair is [1 0] --
  inverted, which `within-limit?` refuses -- so an unreported joint fails
  closed through the ordinary path rather than through a special case."
  [store part targets-deg]
  (let [joints (joints-for-part part)
        table (limits-for store part)]
    (boolean
     (and (seq joints)
          (= (count joints) (count targets-deg))
          (every? true?
                  (map (fn [j q]
                         (let [[lo hi] (get table j [1 0])]
                           (within-limit? (deg->millideg q) lo hi)))
                       joints targets-deg))))))

(defn handle-goto-admit
  "Decide a goto, and persist it only if admitted. `data` carries the SDK's
  own arguments: {:part :space :interpolation_mode :duration :target}, where
  :target is the positional joint list in degrees that the Python SDK's
  goto() takes.

  Returns [body status]: 201 with the accepted GoToRequest, or 409 with the
  reason the core gave. 409 rather than 400 -- the request is well-formed;
  the robot's state is what refuses it."
  [store data]
  (let [part (:part data)
        space (or (:space data) "joint")
        interp (as-int (:interpolation_mode data))
        duration-ms (s->ms (:duration data))
        targets (vec (:target data))
        joints (joints-for-part part)
        expected (count (or joints []))
        shape (goto-shape-reason (part-ready? store part)
                                 (count targets) expected interp duration-ms)
        limits-ok (joint-limits-ok? store part targets)
        reason (admit-reason shape limits-ok)]
    (if-not (= reason reason-ok)
      [{:error {:message reason
                :type "goto_not_admitted"
                :code (admit-code shape limits-ok)}} 409]
      (let [rec {:id (new-id "reachy_goto")
                 :part part :space space
                 :interpolation_mode interp
                 :duration (as-float (:duration data))
                 :status goal-status-accepted
                 :order_id (inc (count (query store "GoToRequest")))
                 :createdAt (now) :updatedAt (now)}]
        (persist! store "GoToRequest" rec)
        [rec 201]))))

(defn handle-goto-cancel
  "GoToService.CancelGoTo. Reports progress only when it made some: a request
  already terminal, or already cancelling, is returned unchanged with ack
  false rather than reported as newly cancelled."
  [store id]
  (let [rows (query store "GoToRequest" id)]
    (if (empty? rows)
      (not-found)
      (let [rec (first rows)
            from (as-int (:status rec))
            to (status-after-cancel from)]
        (if (= from to)
          [{:ack false :goto rec} 200]
          (let [rec' (assoc rec :status to :updatedAt (now))]
            (persist! store "GoToRequest" rec')
            [{:ack true :goto rec'} 200]))))))

(defn handle-goto-transition
  "Advance a request's GoalStatus. Refuses any walk the lifecycle does not
  admit, including a repeat of a terminal status -- so a duplicated
  completion event cannot be mistaken for progress."
  [store id to]
  (let [rows (query store "GoToRequest" id)]
    (if (empty? rows)
      (not-found)
      (let [rec (first rows)
            from (as-int (:status rec))
            to (as-int to)]
        (if-not (transition-admitted? from to)
          [{:error {:message "transition-not-admitted"
                    :type "invalid_state_transition"
                    :from from :to to}} 409]
          (let [rec' (assoc rec :status to :updatedAt (now))]
            (persist! store "GoToRequest" rec')
            [rec' 200]))))))

(defn handle-safety-gate
  "Evaluate the lidar gate for one LidarSafety row and report BOTH scales.
  Reporting only the governed one would hide a disagreement with the robot;
  reporting only the SDK one would hide the refusal. A caller picks by name."
  [store id]
  (let [rows (query store "LidarSafety" id)]
    (if (empty? rows)
      (not-found)
      (let [r (first rows)
            d (m->mm (:obstacle_distance r))
            s (m->mm (:safety_distance r))
            c (m->mm (:critical_distance r))
            on (true? (:safety_on r))]
        [{:part (:part r)
          :obstacle_detection_status (obstacle-status d s c)
          :sdk_speed_scale_permille (sdk-speed-scale-permille on d s c)
          :governed_speed_scale_permille (governed-speed-scale-permille on d s c)
          :motion_admitted (motion-admitted? on d s c)} 200]))))

(defn healthz []
  [{:status "ok" :actor "pollen_robotics-compat" :tier tier
    :entities entities :parts part-names
    :cores ["reachy-lidar-safety-core" "reachy-goto-core"]} 200])

;; --- WASM runtime registration (kotodama). The runtime host owns the live
;;     Datom log; handlers stay pure folds over a store, so this is G5-clean. ---
(defn start! [] :pollen_robotics-compat/ready)
