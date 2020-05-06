/** \brief Select new trial value for More-Thuente method.
		  * \note Trial Value Selection [More, Thuente 1994], \f$ \psi(\alpha_k) \f$ is used for \f$ f_k \f$ and \f$ g_k \f$
		  * until some value satifies the test \f$ \psi(\alpha_k) \leq 0 \f$ and \f$ \phi'(\alpha_k) \geq 0 \f$
		  * then \f$ \phi(\alpha_k) \f$ is used from then on.
		  * \note Interpolation Minimizer equations from Optimization Theory and Methods: Nonlinear Programming By Wenyu Sun, Ya-xiang Yuan (89-100).
		  * \param[in] a_l first endpoint of interval \f$ I \f$, \f$ \alpha_l \f$ in Moore-Thuente (1994)
		  * \param[in] f_l value at first endpoint, \f$ f_l \f$ in Moore-Thuente (1994)
		  * \param[in] g_l derivative at first endpoint, \f$ g_l \f$ in Moore-Thuente (1994)
		  * \param[in] a_u second endpoint of interval \f$ I \f$, \f$ \alpha_u \f$ in Moore-Thuente (1994)
		  * \param[in] f_u value at second endpoint, \f$ f_u \f$ in Moore-Thuente (1994)
		  * \param[in] g_u derivative at second endpoint, \f$ g_u \f$ in Moore-Thuente (1994)
		  * \param[in] a_t previous trial value, \f$ \alpha_t \f$ in Moore-Thuente (1994)
		  * \param[in] f_t value at previous trial value, \f$ f_t \f$ in Moore-Thuente (1994)
		  * \param[in] g_t derivative at previous trial value, \f$ g_t \f$ in Moore-Thuente (1994)
		  * \return new trial value
		  */
		  