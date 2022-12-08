#ifndef STELLA_VSLAM_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_H
#define STELLA_VSLAM_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_H

namespace stella_vslam {

namespace data {
class map_database;
} // namespace data

namespace optimize {

class global_bundle_adjuster {
public:
    /**
     * Constructor
     * @param map_db
     * @param num_iter
     * @param use_huber_kernel
     */
    explicit global_bundle_adjuster(data::map_database* map_db, const unsigned int num_iter = 10, const bool use_huber_kernel = true);

    /**
     * Destructor
     */
    virtual ~global_bundle_adjuster() = default;

    void optimize_for_initialization(bool* const force_stop_flag, bool* camera_was_modified) const;

    /**
     * Perform optimization
     * @param optimized_keyfrm_ids
     * @param optimized_landmark_ids
     * @param lm_to_pos_w_after_global_BA
     * @param keyfrm_to_pose_cw_after_global_BA
     * @param force_stop_flag
     * @return false if aborted
     */
    bool optimizeGlobal(std::unordered_set<unsigned int>& optimized_keyfrm_ids,
                  std::unordered_set<unsigned int>& optimized_landmark_ids,
                  eigen_alloc_unord_map<unsigned int, Vec3_t>& lm_to_pos_w_after_global_BA,
                  eigen_alloc_unord_map<unsigned int, Mat44_t>& keyfrm_to_pose_cw_after_global_BA,
                        bool* const force_stop_flag, int num_iter, bool general_bundle, bool* camera_was_modified) const;

private:
    //! map database
    const data::map_database* map_db_;

    //! number of iterations of optimization
    unsigned int num_iter_;

    //! use Huber loss or not
    const bool use_huber_kernel_;
};

} // namespace optimize
} // namespace stella_vslam

#endif // STELLA_VSLAM_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_H
