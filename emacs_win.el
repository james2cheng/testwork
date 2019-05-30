(set-default-font "Consolas")
(set-fontset-font "fontset-default" 'chinese-gbk "微软雅黑")

(setq face-font-rescale-alist '(("宋体" . 1.2)
                ("微软雅黑" . 1.1)
                ))

;; 设置垃圾回收，在 Windows 下，emacs25 版本会频繁出发垃圾回收，所以需要设置
(when (eq system-type 'windows-nt) 
(setq gc-cons-threshold (* 512 1024 1024)) 
(setq gc-cons-percentage 0.5) 
(run-with-idle-timer 5 t #'garbage-collect)
;; 显示垃圾回收信息，这个可以作为调试用;; 
(setq garbage-collection-messages t))
