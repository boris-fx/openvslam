#ifndef STELLA_VSLAM_MODULE_LOOP_BUNDLE_ADJUSTER_H
#define STELLA_VSLAM_MODULE_LOOP_BUNDLE_ADJUSTER_H

#include <mutex>

namespace stella_vslam {

class mapping_module;

namespace data {
class keyframe;
class map_database;
} // namespace data

namespace module {

class loop_bundle_adjuster {
public:
    /**
     * Constructor
     */
    explicit loop_bundle_adjuster(data::map_database* map_db, const unsigned int num_iter = 10);

    /**
     * Destructor
     */
    ~loop_bundle_adjuster() = default;

    /**
     * Set the mapping module
     */
    void set_mapping_module(mapping_module* mapper);

    /**
     * Abort loop BA externally
     */
    void abort();

    /**
     * Loop BA is running or not
     */
    bool is_running() const;

    /**
     * Run loop BA
     */
    void optimize(const std::shared_ptr<data::keyframe>& curr_keyfrm, int num_iter, bool general_bundle, bool* camera_was_modified);

    int num_iter() const { return num_iter_; }
    void set_num_iter(int num_iter) { num_iter_ = num_iter; }

    void set_general_bundle(bool general_bundle) { general_bundle_ = general_bundle; }

private:
    //! map database
    data::map_database* map_db_ = nullptr;

    //! mapping module
    mapping_module* mapper_ = nullptr;

    //! number of iteration for optimization
    unsigned int num_iter_ = 10;

    //! true for a general bundle, rather than a loop closing bundle
    bool general_bundle_ = false;

    //-----------------------------------------
    // thread management

    //! mutex for access to pause procedure
    mutable std::mutex mtx_thread_;

    //! flag to abort loop BA
    bool abort_loop_BA_ = false;

    //! flag which indicates loop BA is running or not
    bool loop_BA_is_running_ = false;
};

} // namespace module
} // namespace stella_vslam

#endif // STELLA_VSLAM_MODULE_LOOP_BUNDLE_ADJUSTER_H
