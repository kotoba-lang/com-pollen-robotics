(ns pollen-robotics.main-test
  "Contract + behavioural + core-parity test for the pollen_robotics-compat
  L5 actor.

  Three things are checked here, and they are not the same thing:

  1. CONTRACT -- the CRUD surface the schema implies, exercised against the
     in-memory Datom-log store.
  2. BEHAVIOUR -- the refusable commands (goto admission, cancellation,
     GoalStatus transitions, the lidar gate), with the expected answers
     written out by hand so the corpus is a third opinion rather than a
     restatement of the implementation.
  3. PARITY -- `cores/*.cljk` against the oracle in `main.cljc`, by reading
     the core SOURCE and pinning every wire code and reason literal it
     declares.

  What (3) is, precisely: a constant-and-export parity check, not an
  execution parity check. It catches the drift that actually happens -- a
  reordered proto enum, a renamed reason, an export that disappeared -- and
  it does not need the Kotoba compiler on the classpath, so it runs in this
  repo's CI on every push. Executing the compiled KIR against this oracle is
  the stronger check and belongs where the compiler lives; see README
  \"Running the execution parity\". Neither replaces the other, and this file
  does not claim the one it is not doing.

  JVM: `clojure -M:test`. nbb: `npm test`."
  (:require [clojure.test :as t :refer [deftest is testing]]
            [clojure.string :as str]
            [pollen-robotics.main :as m]
            #?(:cljs ["fs" :as fs])))

;; ── reading the cores ────────────────────────────────────────────────────

(defn- read-text [path]
  #?(:clj (slurp path)
     :cljs (.readFileSync fs path "utf8")))

(def ^:private lidar-src (read-text "cores/reachy_lidar_safety_core.cljk"))
(def ^:private goto-src (read-text "cores/reachy_goto_core.cljk"))

(defn- i64-consts
  "Every `(defn name [] :i64 N)` in a core: its nullary integer constants."
  [src]
  (into {} (map (fn [[_ n v]] [n (#?(:clj Long/parseLong :cljs js/parseInt) v)])
                (re-seq #"\(defn\s+([a-z0-9?<>=*+/-]+)\s+\[\]\s+:i64\s+(-?\d+)\)" src))))

(defn- string-consts
  "Every `(defn name [] :string \"S\")` in a core: its nullary string constants."
  [src]
  (into {} (map (fn [[_ n v]] [n v])
                (re-seq #"\(defn\s+([a-z0-9?<>=*+/-]+)\s+\[\]\s+:string\s+\"([^\"]*)\"\)" src))))

(defn- exports [src]
  (set (-> (re-find #"\(:export\s+\[([^\]]+)\]\)" src) second
           str/trim (str/split #"\s+"))))

;; ── contract ─────────────────────────────────────────────────────────────

(defn- dummy [field coerce]
  (case (get coerce field) :int 1 :float 1.0 :bool true (name field)))

(defn- full-record [{:keys [required coerce]}]
  (into {} (map (fn [f] [f (dummy f coerce)]) required)))

(deftest route-surface
  (is (= (* 5 (count m/entity-specs)) (count m/crud-routes)))
  (is (= (+ (count m/crud-routes) (count m/command-routes)) (count m/routes)))
  (doseq [{:keys [plural]} m/entity-specs]
    (let [paths (set (map (juxt :method :path) m/crud-routes))
          base (str "/v1/" plural)]
      (is (contains? paths ["POST" base]))
      (is (contains? paths ["GET" base]))
      (is (contains? paths ["GET" (str base "/{id}")]))
      (is (contains? paths ["PATCH" (str base "/{id}")]))
      (is (contains? paths ["DELETE" (str base "/{id}")]))))
  (testing "every command route names an entity the schema declares"
    (doseq [{:keys [entity]} m/command-routes]
      (is (contains? (set m/entities) entity)))))

(deftest crud-roundtrip
  (doseq [{:keys [entity id-prefix] :as spec} m/entity-specs]
    (let [s (m/fresh-store)
          [rec status] (m/handle-create s entity (full-record spec))]
      (is (= 201 status) (str entity " create"))
      (is (str/starts-with? (:id rec) (str id-prefix "_")) (str entity " id-prefix"))
      (is (= [rec 200] (m/handle-get s entity (:id rec) {})) (str entity " get"))
      (is (= (:id rec) (:id (first (m/handle-update s entity (:id rec) {})))))
      (is (= 200 (second (m/handle-delete s entity (:id rec)))))
      (is (= 404 (second (m/handle-get s entity (:id rec) {})))))))

(deftest validation
  (doseq [{:keys [entity required] :as spec} m/entity-specs]
    (when (seq required)
      (let [s (m/fresh-store)]
        (is (= 400 (second (m/handle-create s entity {}))) (str entity " missing-required"))
        (is (= 400 (second (m/handle-create s entity (assoc (full-record spec) :__bogus__ 1))))
            (str entity " unknown"))))))

(deftest coercion
  (doseq [{:keys [entity coerce] :as spec} m/entity-specs]
    (when (seq coerce)
      (let [s (m/fresh-store) [rec _] (m/handle-create s entity (full-record spec))]
        (doseq [[f kind] coerce]
          (is (case kind
                :int (integer? (get rec f))
                :float (number? (get rec f))
                :bool (boolean? (get rec f))
                true)
              (str entity "/" (name f))))))))

(deftest emit-facts-namespacing
  (let [f (m/emit-facts "GoToRequest" {:part "r_arm" :status 2})]
    (is (= "r_arm" (get f "pollen_robotics.GoToRequest/part")))
    (is (= 2 (get f "pollen_robotics.GoToRequest/status")))))

(deftest healthz
  (let [[body status] (m/healthz)]
    (is (= 200 status))
    (is (= "pollen_robotics-compat" (:actor body)))
    (is (= (set m/entities) (set (:entities body))))
    (is (= #{"l_arm" "r_arm" "head" "l_hand" "r_hand" "mobile_base"}
           (set (:parts body))))))

;; ── behaviour: the lidar gate ────────────────────────────────────────────
;;
;; Distances in millimetres. safety ring 500mm, critical ring 200mm unless a
;; row says otherwise. Expected answers written out, not computed.

(deftest lidar-gate
  (testing "classification"
    (is (= m/status-no-object (m/obstacle-status 900 500 200)))
    (is (= m/status-slowdown  (m/obstacle-status 500 500 200)) "the ring itself is inside")
    (is (= m/status-slowdown  (m/obstacle-status 350 500 200)))
    (is (= m/status-stop      (m/obstacle-status 200 500 200)) "the ring itself is inside")
    (is (= m/status-stop      (m/obstacle-status 0 500 200)))
    (is (= m/status-detection-error (m/obstacle-status -1 500 200)) "no reading is not 'clear'"))
  (testing "a misconfigured ring pair classifies nothing"
    (is (= m/status-detection-error (m/obstacle-status 350 200 500)) "inverted")
    (is (= m/status-detection-error (m/obstacle-status 350 500 0))   "critical at zero")
    (is (= m/status-detection-error (m/obstacle-status 350 500 500)) "coincident"))
  (testing "the ramp is linear between the rings and never rounds up"
    (is (= 0    (m/ramp-permille 200 500 200)))
    (is (= 500  (m/ramp-permille 350 500 200)))
    (is (= 1000 (m/ramp-permille 500 500 200)))
    (is (= 333  (m/ramp-permille 300 500 200)) "100/300 truncates down, not to 334"))
  (testing "safety_on off reproduces the robot, and does not survive the governor"
    (is (= 1000 (m/sdk-speed-scale-permille false 100 500 200)))
    (is (= 0    (m/governed-speed-scale-permille false 100 500 200)))
    (is (= 1000 (m/sdk-speed-scale-permille false -1 500 200)))
    (is (= 0    (m/governed-speed-scale-permille false -1 500 200))
        "an unreadable lidar is not licence to move")
    (is (= 1000 (m/governed-speed-scale-permille false 350 500 200))
        "the switch may relax a SLOWDOWN"))
  (testing "with safety on the two agree"
    (doseq [d [-1 0 200 300 350 500 900]]
      (is (= (m/sdk-speed-scale-permille true d 500 200)
             (m/governed-speed-scale-permille true d 500 200))
          (str "d=" d))))
  (testing "motion-admitted? is the governed scale, thresholded"
    (is (false? (m/motion-admitted? true 200 500 200)))
    (is (false? (m/motion-admitted? true -1 500 200)))
    (is (true?  (m/motion-admitted? true 350 500 200)))))

(deftest safety-gate-handler
  (let [s (m/fresh-store)
        [row _] (m/handle-create s "LidarSafety"
                                 {:part "mobile_base" :safety_on true
                                  :safety_distance 0.5 :critical_distance 0.2
                                  :obstacle_distance 0.15})
        [body status] (m/handle-safety-gate s (:id row))]
    (is (= 200 status))
    (is (= m/status-stop (:obstacle_detection_status body)))
    (is (= 0 (:governed_speed_scale_permille body)))
    (is (false? (:motion_admitted body))))
  (is (= 404 (second (m/handle-safety-gate (m/fresh-store) "nope")))))

;; ── behaviour: goto ──────────────────────────────────────────────────────

(defn- seeded-arm-store []
  (let [s (m/fresh-store)]
    (m/handle-create s "Part" {:name "r_arm" :kind "arm" :activated true})
    (doseq [j m/arm-joints]
      (m/handle-create s "JointLimit" {:part "r_arm" :joint j :min -180.0 :max 180.0}))
    s))

(def ^:private zeros7 [0.0 0.0 0.0 0.0 0.0 0.0 0.0])

(defn- admit [s data]
  (m/handle-goto-admit s (merge {:part "r_arm" :space "joint"
                                 :interpolation_mode m/interpolation-minimum-jerk
                                 :duration 2.0 :target zeros7}
                                data)))

(deftest goto-admission
  (testing "a well-formed goto within the reported limits is accepted"
    (let [[rec status] (admit (seeded-arm-store) {})]
      (is (= 201 status))
      (is (= m/goal-status-accepted (:status rec)))
      (is (= 1 (:order_id rec)))))
  (testing "each refusal names its own reason"
    (let [s (seeded-arm-store)]
      (is (= "joint-count-mismatch"
             (get-in (first (admit s {:target [0.0 0.0 0.0]})) [:error :message])))
      (is (= "interpolation-mode-not-admitted"
             (get-in (first (admit s {:interpolation_mode m/interpolation-none})) [:error :message]))
          "proto3's unset zero value is not a default of minimum-jerk")
      (is (= "duration-out-of-range"
             (get-in (first (admit s {:duration 0.0})) [:error :message]))
          "zero duration is the goal_position path, not a goto")
      (is (= "duration-out-of-range"
             (get-in (first (admit s {:duration 601.0})) [:error :message])))
      (is (= "joint-limit-exceeded"
             (get-in (first (admit s {:target [200.0 0.0 0.0 0.0 0.0 0.0 0.0]})) [:error :message])))
      (is (= 409 (second (admit s {:target [200.0 0.0 0.0 0.0 0.0 0.0 0.0]}))))))
  (testing "a part the robot never reported is not ready"
    (is (= "part-not-ready"
           (get-in (first (admit (m/fresh-store) {})) [:error :message]))))
  (testing "a part reported but not activated is not ready"
    (let [s (m/fresh-store)]
      (m/handle-create s "Part" {:name "r_arm" :kind "arm" :activated false})
      (is (= "part-not-ready" (get-in (first (admit s {})) [:error :message])))))
  (testing "a joint whose limit was never reported fails closed"
    (let [s (m/fresh-store)]
      (m/handle-create s "Part" {:name "r_arm" :kind "arm" :activated true})
      (doseq [j (butlast m/arm-joints)]
        (m/handle-create s "JointLimit" {:part "r_arm" :joint j :min -180.0 :max 180.0}))
      (is (= "joint-limit-exceeded" (get-in (first (admit s {})) [:error :message])))))
  (testing "a part that takes no joint-space goto refuses one"
    (let [s (m/fresh-store)]
      (m/handle-create s "Part" {:name "mobile_base" :kind "base" :activated true})
      (is (= 409 (second (m/handle-goto-admit
                          s {:part "mobile_base" :space "joint"
                             :interpolation_mode m/interpolation-linear
                             :duration 2.0 :target []}))))))
  (testing "nothing is persisted by a refusal"
    (let [s (seeded-arm-store)]
      (admit s {:duration 0.0})
      (is (empty? (m/query s "GoToRequest"))))))

(deftest goto-limits-are-the-robots
  (testing "the same target is admitted or refused by the limits the robot reported"
    (let [s (m/fresh-store)]
      (m/handle-create s "Part" {:name "head" :kind "head" :activated true})
      (doseq [j m/neck-joints]
        (m/handle-create s "JointLimit" {:part "head" :joint j :min -10.0 :max 10.0}))
      (let [req {:part "head" :space "joint"
                 :interpolation_mode m/interpolation-linear :duration 1.0}]
        (is (= 201 (second (m/handle-goto-admit s (assoc req :target [5.0 0.0 0.0])))))
        (is (= 409 (second (m/handle-goto-admit s (assoc req :target [15.0 0.0 0.0])))))))))

(deftest goto-lifecycle
  (testing "the admitted walk"
    (is (true? (m/transition-admitted? m/goal-status-accepted m/goal-status-executing)))
    (is (true? (m/transition-admitted? m/goal-status-executing m/goal-status-succeeded)))
    (is (true? (m/transition-admitted? m/goal-status-executing m/goal-status-canceling)))
    (is (true? (m/transition-admitted? m/goal-status-canceling m/goal-status-canceled))))
  (testing "terminal is terminal, including a repeat of itself"
    (doseq [t [m/goal-status-succeeded m/goal-status-canceled m/goal-status-aborted]]
      (is (false? (m/transition-admitted? t t)) "a duplicated event is not progress")
      (is (false? (m/transition-admitted? t m/goal-status-executing)))))
  (testing "no skipping"
    (is (false? (m/transition-admitted? m/goal-status-accepted m/goal-status-succeeded)))
    (is (false? (m/transition-admitted? m/goal-status-canceling m/goal-status-succeeded))))
  (testing "cancel reports progress only when it made some"
    (let [s (seeded-arm-store)
          [rec _] (admit s {})
          [b1 _] (m/handle-goto-cancel s (:id rec))
          [b2 _] (m/handle-goto-cancel s (:id rec))]
      (is (true? (:ack b1)))
      (is (= m/goal-status-canceling (get-in b1 [:goto :status])))
      (is (false? (:ack b2)) "already cancelling is not newly cancelled")
      (is (= m/goal-status-canceling (get-in b2 [:goto :status])))))
  (testing "the transition handler refuses what the core refuses"
    (let [s (seeded-arm-store)
          [rec _] (admit s {})]
      (is (= 200 (second (m/handle-goto-transition s (:id rec) m/goal-status-executing))))
      (is (= 200 (second (m/handle-goto-transition s (:id rec) m/goal-status-succeeded))))
      (is (= 409 (second (m/handle-goto-transition s (:id rec) m/goal-status-executing))))))
  (is (= 404 (second (m/handle-goto-cancel (m/fresh-store) "nope"))))
  (is (= 404 (second (m/handle-goto-transition (m/fresh-store) "nope" 3)))))

;; ── host boundary ────────────────────────────────────────────────────────

(deftest fixed-point-conversion
  (is (= 90000 (m/deg->millideg 90.0)))
  (is (= -90000 (m/deg->millideg -90.0)))
  (is (= 500 (m/m->mm 0.5)))
  (is (= 2000 (m/s->ms 2.0)))
  (is (= 1 (m/deg->millideg 0.0005)) "half rounds up, on both hosts")
  (is (= 0 (m/deg->millideg 0.0004))))

;; ── parity: cores/*.cljk against this file's oracle ──────────────────────
;;
;; The map below is the only place the two vocabularies meet, and it is
;; exhaustive on purpose: a constant the core declares and this map does not
;; name fails the test rather than going unchecked.

(def ^:private lidar-i64-names
  {"status-detection-error" m/status-detection-error
   "status-no-object"       m/status-no-object
   "status-slowdown"        m/status-slowdown
   "status-stop"            m/status-stop
   "scale-full"             m/scale-full
   "scale-stop"             m/scale-stop})

(def ^:private goto-i64-names
  {"interpolation-none"         m/interpolation-none
   "interpolation-linear"       m/interpolation-linear
   "interpolation-minimum-jerk" m/interpolation-minimum-jerk
   "status-none"       m/goal-status-none
   "status-unknown"    m/goal-status-unknown
   "status-accepted"   m/goal-status-accepted
   "status-executing"  m/goal-status-executing
   "status-canceling"  m/goal-status-canceling
   "status-succeeded"  m/goal-status-succeeded
   "status-canceled"   m/goal-status-canceled
   "status-aborted"    m/goal-status-aborted
   "arm-joint-count"   m/arm-joint-count
   "neck-joint-count"  m/neck-joint-count
   "min-duration-ms"   m/min-duration-ms
   "max-duration-ms"   m/max-duration-ms})

(def ^:private goto-string-names
  {"reason-ok"             m/reason-ok
   "reason-part-not-ready" m/reason-part-not-ready
   "reason-joint-count"    m/reason-joint-count
   "reason-interpolation"  m/reason-interpolation
   "reason-duration"       m/reason-duration
   "reason-joint-limit"    m/reason-joint-limit})

(deftest core-constants-parity
  (testing "the lidar core's wire codes are this oracle's"
    (let [found (i64-consts lidar-src)]
      (is (seq found) "the core source was read")
      (is (= (set (keys lidar-i64-names)) (set (keys found)))
          "a constant was added or removed in the core without being mapped here")
      (doseq [[n v] found]
        (is (= (get lidar-i64-names n) v) (str "lidar core " n)))))
  (testing "the goto core's wire codes are this oracle's"
    (let [found (i64-consts goto-src)]
      (is (= (set (keys goto-i64-names)) (set (keys found)))
          "a constant was added or removed in the core without being mapped here")
      (doseq [[n v] found]
        (is (= (get goto-i64-names n) v) (str "goto core " n)))))
  (testing "the goto core's reason literals are this oracle's"
    (let [found (string-consts goto-src)]
      (is (= (set (keys goto-string-names)) (set (keys found))))
      (doseq [[n v] found]
        (is (= (get goto-string-names n) v) (str "goto core " n))))))

(deftest core-exports-parity
  (testing "every decision the lidar core exports has an oracle here"
    (is (= #{"status-detection-error" "status-no-object" "status-slowdown" "status-stop"
             "scale-full" "scale-stop" "reading-valid?" "distances-sane?"
             "obstacle-status" "ramp-permille"
             "sdk-speed-scale-permille" "governed-speed-scale-permille"
             "motion-admitted?"}
           (exports lidar-src))))
  (testing "every decision the goto core exports has an oracle here"
    (is (= #{"interpolation-none" "interpolation-linear" "interpolation-minimum-jerk"
             "status-none" "status-unknown" "status-accepted" "status-executing"
             "status-canceling" "status-succeeded" "status-canceled" "status-aborted"
             "arm-joint-count" "neck-joint-count" "min-duration-ms" "max-duration-ms"
             "reason-ok" "reason-part-not-ready" "reason-joint-count"
             "reason-interpolation" "reason-duration" "reason-joint-limit"
             "interpolation-admitted?" "duration-admitted?" "joint-count-ok?"
             "within-limit?" "goto-shape-reason" "admit-reason" "admitted?" "admit-code"
             "terminal?" "cancellable?" "active?" "status-after-cancel"
             "transition-admitted?"}
           (exports goto-src)))))

(deftest joint-counts-match-the-protos
  (is (= m/arm-joint-count (count m/arm-joints)))
  (is (= m/neck-joint-count (count m/neck-joints)))
  (is (= ["shoulder.pitch" "shoulder.roll" "elbow.yaw" "elbow.pitch"
          "wrist.roll" "wrist.pitch" "wrist.yaw"]
         m/arm-joints)
      "ArmJoints declaration order is part of the contract -- goto() is positional"))

#?(:clj (defn -main [& _]
          (let [{:keys [fail error]} (t/run-tests 'pollen-robotics.main-test)]
            (System/exit (if (pos? (+ fail error)) 1 0)))))
